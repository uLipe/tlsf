//
// @file MemManager.c
// @brief modulo gerenciador de memoria rapido e deterministico
//        para embedded, baseado no algoritmo TLSF de Miguel Masmano
//


//

/* 
 * Two Levels Segregate Fit memory allocator (TLSF)
 * Version 2.4.4
 *
 * Written by Miguel Masmano Tello <mimastel@doctor.upv.es>
 *
 * Thanks to Ismael Ripoll for his suggestions and reviews
 *
 * Copyright (C) 2008, 2007, 2006, 2005, 2004
 *
 * This code is released using a dual license strategy: GPL/LGPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of the GNU General Public License Version 2.0
 * Released under the terms of the GNU Lesser General Public License Version 2.1
 *
 */
 
 /*
 * Code contributions:
 *
 * (Jul 28 2007)  Herman ten Brugge <hermantenbrugge@home.nl>:
 *
 * - Add 64 bit support. It now runs on x86_64 and solaris64.
 * - I also tested this on vxworks/32and solaris/32 and i386/32 processors.
 * - Remove assembly code. I could not measure any performance difference 
 *   on my core2 processor. This also makes the code more portable.
 * - Moved defines/typedefs from tlsf.h to tlsf.c
 * - Changed MIN_BLOCK_SIZE to sizeof (free_ptr_t) and BHDR_OVERHEAD to 
 *   (sizeof (bhdr_t) - MIN_BLOCK_SIZE). This does not change the fact 
 *    that the minumum size is still sizeof 
 *   (bhdr_t).
 * - Changed all C++ comment style to C style. (// -> /.* ... *./)
 * - Used ls_bit instead of ffs and ms_bit instead of fls. I did this to 
 *   avoid confusion with the standard ffs function which returns 
 *   different values.
 * - Created set_bit/clear_bit fuctions because they are not present 
 *   on x86_64.
 * - Added locking support + extra file target.h to show how to use it.
 * - Added get_used_size function (REMOVED in 2.4)
 * - Added rtl_realloc and rtl_calloc function
 * - Implemented realloc clever support.
 * - Added some test code in the example directory.
 *        
 *
 * (Oct 23 2006) Adam Scislowicz: 
 *
 * - Support for ARMv5 implemented
 *
 */
 
/* Code contributions:
* (2015) Felipe Neves: 
 * - added efficient ffs /fls based on armv7-m specifics
 *
 * 
 */
 #include <stdio.h>
 #include <string.h>
 #include "tlsf.h"
 
//
// Macros usadas para implementacado do sistema de estatistica:
//
#define	TLSF_ADD_SIZE(tlsf, b) do {									\
		tlsf->used_size += (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;	\
		if (tlsf->used_size > tlsf->max_size) 						\
			tlsf->max_size = tlsf->used_size;						\
		} while(0)

#define	TLSF_REMOVE_SIZE(tlsf, b) do {								\
		tlsf->used_size -= (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;	\
	} while(0)

//
//	Macros e definicoes da estrutura do TFSL
//	
	
#define BLOCK_ALIGN (sizeof(void *) * 2)										//Alinhamento minimo de bloco (tamanho minimo de 4bytes)

#define MAX_FLI									(30)
#define MAX_LOG2_SLI						(5)
#define MAX_SLI									(1 << MAX_LOG2_SLI)     		//Calculador de posicoes maximas dos bitmaps

#define FLI_OFFSET							(6)     										//Profundidade do bitmap de primeiro nivel
#define SMALL_BLOCK							(128)
#define REAL_FLI								(MAX_FLI - FLI_OFFSET)
#define MIN_BLOCK_SIZE					(sizeof (free_ptr_t))
#define BHDR_OVERHEAD						(sizeof (bhdr_t) - MIN_BLOCK_SIZE)
#define TLSF_SIGNATURE					(0x2A59FA59)

#define	PTR_MASK								(sizeof(void *) - 1)
#define BLOCK_SIZE							(0xFFFFFFFF - PTR_MASK)

#define GET_NEXT_BLOCK(_addr, _r) ((bhdr_t *) ((uint8_t *) (_addr) + (_r)))
#define	MEM_ALIGN		  					((BLOCK_ALIGN) - 1)
#define ROUNDUP_SIZE(_r)        (((_r) + MEM_ALIGN) & ~MEM_ALIGN)
#define ROUNDDOWN_SIZE(_r)      ((_r) & ~MEM_ALIGN)
#define ROUNDUP(_x, _v)         ((((~(_x)) + 1) & ((_v)-1)) + (_x))

//
// Estados dos blocos de memoria
//
#define BLOCK_STATE							(0x1)
#define PREV_STATE							(0x2)
#define FREE_BLOCK							(0x1)
#define USED_BLOCK							(0x0)
#define PREV_FREE								(0x2)
#define PREV_USED								(0x0)

//
// Area size se utilizado com SBRK:
//
#define DEFAULT_AREA_SIZE (1024*10)


//
// Log & asserts:
//
#define PRINT_MSG(fmt, args...) printf(fmt, ## args)
#define ERROR_MSG(fmt, args...) printf(fmt, ## args)

//
// Heap linked list cast:
//
typedef struct free_ptr_struct 
{
    struct bhdr_struct *prev;
    struct bhdr_struct *next;
} free_ptr_t;

//
// TFSL block header:
//
typedef struct bhdr_struct 
{
    struct bhdr_struct *prev_hdr;
    size_t size;                
    union 
		{
        struct free_ptr_struct free_ptr;
        uint8_t buffer[1];
    } ptr;
		
}bhdr_t;

//
// Area info:
//
typedef struct area_info_struct 
{
    bhdr_t *end;
    struct area_info_struct *next;
} area_info_t;

//
// Estrutura TFSL completa aresponsavel por gerenciar o heap:
//
typedef struct TLSF_struct 
{
	uint32_t tlsf_signature;
	size_t used_size;
	size_t max_size;

	area_info_t *area_head;

	//
	// Bitmap de acesso aos blocos:
	//
	uint32_t fl_bitmap;
	uint32_t sl_bitmap[REAL_FLI];
	
	//
	// Matrix de ponteiros para a linked buddy list:
	//
	bhdr_t *matrix[REAL_FLI][MAX_SLI];
} tlsf_t;


//
// mecanismo de Thread safe 
//


//
// Forward references de funcoes internas para busca de blocos:
//
static __inline void set_bit(int32_t nr, uint32_t * addr);
static __inline void clear_bit(int32_t nr, uint32_t * addr);
static __inline int32_t ls_bit(int32_t x);
static __inline int32_t ms_bit(int32_t x);
static __inline void MAPPING_SEARCH(size_t * _r, int32_t *_fl, int32_t *_sl);
static __inline void MAPPING_INSERT(size_t _r, int32_t *_fl, int32_t *_sl);
static __inline bhdr_t *FIND_SUITABLE_BLOCK(tlsf_t * _tlsf, int32_t *_fl, int32_t *_sl);
static __inline bhdr_t *process_area(void *area, size_t size);
static size_t init_memory_pool(size_t mem_pool_size, void *mem_pool);
static size_t add_new_area(void *area, size_t area_size, void *mem_pool);
static size_t get_used_size(void *mem_pool);
static size_t get_max_size(void *mem_pool);
static void destroy_memory_pool(void *mem_pool);
static void *malloc_ex(size_t size, void *mem_pool);	
static void free_ex(void *ptr, void *mem_pool);
//
// ls_bit()
//
static __inline int32_t ls_bit(int32_t i)
{
	//Usa a funcao otimizada contida em bitman.c:
	return(fls(i));
}

//
// ms_bit():
//
static __inline int32_t ms_bit(int32_t i)
{
	//usa funcao otimizada contida em bitman.c
	return(ffs(i));
}

//
// set_bit()
//
static __inline void set_bit(int32_t nr, uint32_t * addr)
{
    addr[nr >> 5] |= 1 << (nr & 0x1f);
}

//
// clear bit:
//
static __inline void clear_bit(int32_t nr, uint32_t * addr)
{
    addr[nr >> 5] &= ~(1 << (nr & 0x1f));
}

//
// MAPPING_SEARCH()
//
static __inline void MAPPING_SEARCH(size_t * _r, int32_t *_fl, int32_t *_sl)
{
    int32_t _t;
		
		//
		// Esse helper tem por funcao a partir do tamanho do bloco
		// de memoria desejado, mapear via um bitmap de dois niveis
		// a buddy list que pode conter o bloco de tamanho adequado:
		//
    if (*_r < SMALL_BLOCK) 
		{
        *_fl = 0;
        *_sl = *_r / (SMALL_BLOCK / MAX_SLI);
    } 
		else 
		{
        _t = (1 << (ms_bit(*_r) - MAX_LOG2_SLI)) - 1;
        *_r = *_r + _t;
        *_fl = ms_bit(*_r);
        *_sl = (*_r >> (*_fl - MAX_LOG2_SLI)) - MAX_SLI;
        *_fl -= FLI_OFFSET;
        *_r &= ~_t;
    }
}

//
// MAPPING_INSERT:
//
static __inline void MAPPING_INSERT(size_t _r, int32_t *_fl, int32_t *_sl)
{
	
		//
		// De forma similar ao search, o mapping insert
		// em funcao do tamanho do bloco passado (em um free) 
		// mapeia a buddy list (atraves dos bitmaps) para saber
		// a posicao nesta em qual o blcoo deve ser devolvido.
    if (_r < SMALL_BLOCK)
		{
        *_fl = 0;
        *_sl = _r / (SMALL_BLOCK / MAX_SLI);
    } 
		else 
		{
        *_fl = ms_bit(_r);
        *_sl = (_r >> (*_fl - MAX_LOG2_SLI)) - MAX_SLI;
        *_fl -= FLI_OFFSET;
    }
}

//
// FIND_SUITABLE_BLOCK()
//
static __inline bhdr_t *FIND_SUITABLE_BLOCK(tlsf_t * _tlsf, int32_t *_fl, int32_t *_sl)
{
    uint32_t _tmp = _tlsf->sl_bitmap[*_fl] & (~0 << *_sl);
    bhdr_t *_b = NULL;

	
		//
		// A partir das posicoes do bitmap obtidas executa a estrategia
		// de good fit para pegar da buddylist o melhor bloco para o size
		// desejado conforme a politica do good fit:
    if (_tmp)
		{
        *_sl = ls_bit(_tmp);
        _b = _tlsf->matrix[*_fl][*_sl];
    } 
		else 
		{
        *_fl = ls_bit(_tlsf->fl_bitmap & (~0 << (*_fl + 1)));
        
				if (*_fl > 0) 
				{         
            *_sl = ls_bit(_tlsf->sl_bitmap[*_fl]);
            _b = _tlsf->matrix[*_fl][*_sl];
        }
    }
    return _b;
}

//
// Remove o block header do stream de memoria obtido:
//
#define EXTRACT_BLOCK_HDR(_b, _tlsf, _fl, _sl) do {					\
		_tlsf -> matrix [_fl] [_sl] = _b -> ptr.free_ptr.next;		\
		if (_tlsf -> matrix[_fl][_sl])								\
			_tlsf -> matrix[_fl][_sl] -> ptr.free_ptr.prev = NULL;	\
		else {														\
			clear_bit (_sl, &_tlsf -> sl_bitmap [_fl]);				\
			if (!_tlsf -> sl_bitmap [_fl])							\
				clear_bit (_fl, &_tlsf -> fl_bitmap);				\
		}															\
		_b -> ptr.free_ptr.prev =  NULL;				\
		_b -> ptr.free_ptr.next =  NULL;				\
	}while(0)


//
// Faz unlink do bloco de memoria da buddy list:
//	
#define EXTRACT_BLOCK(_b, _tlsf, _fl, _sl) do {							\
		if (_b -> ptr.free_ptr.next)									\
			_b -> ptr.free_ptr.next -> ptr.free_ptr.prev = _b -> ptr.free_ptr.prev; \
		if (_b -> ptr.free_ptr.prev)									\
			_b -> ptr.free_ptr.prev -> ptr.free_ptr.next = _b -> ptr.free_ptr.next; \
		if (_tlsf -> matrix [_fl][_sl] == _b) {							\
			_tlsf -> matrix [_fl][_sl] = _b -> ptr.free_ptr.next;		\
			if (!_tlsf -> matrix [_fl][_sl]) {							\
				clear_bit (_sl, &_tlsf -> sl_bitmap[_fl]);				\
				if (!_tlsf -> sl_bitmap [_fl])							\
					clear_bit (_fl, &_tlsf -> fl_bitmap);				\
			}															\
		}																\
		_b -> ptr.free_ptr.prev = NULL;					\
		_b -> ptr.free_ptr.next = NULL;					\
	} while(0)

//
// efetua o link (ou re-link) de um bloco de memoria na buddy list:
//	
#define INSERT_BLOCK(_b, _tlsf, _fl, _sl) do {							\
		_b -> ptr.free_ptr.prev = NULL; \
		_b -> ptr.free_ptr.next = _tlsf -> matrix [_fl][_sl]; \
		if (_tlsf -> matrix [_fl][_sl])									\
			_tlsf -> matrix [_fl][_sl] -> ptr.free_ptr.prev = _b;		\
		_tlsf -> matrix [_fl][_sl] = _b;								\
		set_bit (_sl, &_tlsf -> sl_bitmap [_fl]);						\
		set_bit (_fl, &_tlsf -> fl_bitmap);								\
	} while(0)

	
//
// get_new_area()
//	
static __inline void *get_new_area(size_t * size) 
{
    return ((void *) ~0);
}

//
// process_area()
//
static __inline bhdr_t *process_area(void *area, size_t size)
{
    bhdr_t *b, *lb, *ib;
    area_info_t *ai;

    ib = (bhdr_t *) area;
    ib->size =
        (sizeof(area_info_t) <
         MIN_BLOCK_SIZE) ? MIN_BLOCK_SIZE : ROUNDUP_SIZE(sizeof(area_info_t)) | USED_BLOCK | PREV_USED;
   
		b = (bhdr_t *) GET_NEXT_BLOCK(ib->ptr.buffer, ib->size & BLOCK_SIZE);
    b->size = ROUNDDOWN_SIZE(size - 3 * BHDR_OVERHEAD - (ib->size & BLOCK_SIZE)) | USED_BLOCK | PREV_USED;
    b->ptr.free_ptr.prev = b->ptr.free_ptr.next = 0;
    lb = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    lb->prev_hdr = b;
    lb->size = 0 | USED_BLOCK | PREV_FREE;
    ai = (area_info_t *) ib->ptr.buffer;
    ai->next = 0;
    ai->end = lb;
    return ib;
}

//
// Funcoes high level do alocador de memoria:
//
static uint8_t *mp = NULL;  //Memory pool default.

//
// init_memory_pool():
//
size_t init_memory_pool(size_t mem_pool_size, void *mem_pool)
{
    tlsf_t *tlsf;
    bhdr_t *b, *ib;

		//
		// Checa consistencia do ponteiro da pool:
		//
    if (!mem_pool || !mem_pool_size || mem_pool_size < sizeof(tlsf_t) + BHDR_OVERHEAD * 8) 
	{
        ERROR_MSG("init_memory_pool (): memory_pool invalid\n");
        return -1;
    }

		//
		// Checa se o alinhamento da pool esta correto,
		// deve estar alinhado em pelomenos 4bytes:	 
    if (((unsigned long) mem_pool & PTR_MASK)) 
	{
        ERROR_MSG("init_memory_pool (): mem_pool must be aligned to a word\n");
        return -1;
    }
		
		//
		// Mapeia no formato da estrutura tfsl:
		//
    tlsf = (tlsf_t *) mem_pool;
		
		//
		// Checa se ja foi inicializada:
		//
    if (tlsf->tlsf_signature == TLSF_SIGNATURE) {
        mp = mem_pool;
        b = GET_NEXT_BLOCK(mp, ROUNDUP_SIZE(sizeof(tlsf_t)));
        return b->size & BLOCK_SIZE;
    }

		//
		// Faz o default  mp apontar pra \B4pool:
		//
    mp = mem_pool;

    //
		// Faz zero fill da pool, usa a funcao otimizada contida em optmem.s
		//

		//
		// Inicializa o mapa de memoria da pool:
		//
    tlsf->tlsf_signature = TLSF_SIGNATURE;

    ib = process_area(GET_NEXT_BLOCK
                      (mem_pool, ROUNDUP_SIZE(sizeof(tlsf_t))), ROUNDDOWN_SIZE(mem_pool_size - sizeof(tlsf_t)));
    b = GET_NEXT_BLOCK(ib->ptr.buffer, ib->size & BLOCK_SIZE);
    free_ex(b->ptr.buffer, tlsf);
    tlsf->area_head = (area_info_t *) ib->ptr.buffer;

		//
		// Inicializa sistema de estatistica da pool:
		//
    tlsf->used_size = mem_pool_size - (b->size & BLOCK_SIZE);
    tlsf->max_size = tlsf->used_size;


    return (b->size & BLOCK_SIZE);
}

//
// add_new_area()
//
size_t add_new_area(void *area, size_t area_size, void *mem_pool)
{
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    area_info_t *ptr, *ptr_prev, *ai;
    bhdr_t *ib0, *b0, *lb0, *ib1, *b1, *lb1, *next_b;

    memset(area, 0, area_size);
    ptr = tlsf->area_head;
    ptr_prev = 0;

    ib0 = process_area(area, area_size);
    b0 = GET_NEXT_BLOCK(ib0->ptr.buffer, ib0->size & BLOCK_SIZE);
    lb0 = GET_NEXT_BLOCK(b0->ptr.buffer, b0->size & BLOCK_SIZE);

    while (ptr) 
		{
        ib1 = (bhdr_t *) ((uint8_t *) ptr - BHDR_OVERHEAD);
        b1 = GET_NEXT_BLOCK(ib1->ptr.buffer, ib1->size & BLOCK_SIZE);
        lb1 = ptr->end;

        if ((unsigned long) ib1 == (unsigned long) lb0 + BHDR_OVERHEAD) 
				{
            if (tlsf->area_head == ptr) 
						{
                tlsf->area_head = ptr->next;
                ptr = ptr->next;
            } 
						else 
						{
                ptr_prev->next = ptr->next;
                ptr = ptr->next;
            }

            b0->size =
                ROUNDDOWN_SIZE((b0->size & BLOCK_SIZE) +
                               (ib1->size & BLOCK_SIZE) + 2 * BHDR_OVERHEAD) | USED_BLOCK | PREV_USED;

            b1->prev_hdr = b0;
            lb0 = lb1;

            continue;
        }

        if ((unsigned long) lb1->ptr.buffer == (unsigned long) ib0) 
				{
            if (tlsf->area_head == ptr) 
						{
                tlsf->area_head = ptr->next;
                ptr = ptr->next;
            } 
						else 
						{
                ptr_prev->next = ptr->next;
                ptr = ptr->next;
            }

            lb1->size =
                ROUNDDOWN_SIZE((b0->size & BLOCK_SIZE) +
                               (ib0->size & BLOCK_SIZE) + 2 * BHDR_OVERHEAD) | USED_BLOCK | (lb1->size & PREV_STATE);
            next_b = GET_NEXT_BLOCK(lb1->ptr.buffer, lb1->size & BLOCK_SIZE);
            next_b->prev_hdr = lb1;
            b0 = lb1;
            ib0 = ib1;

            continue;
        }
        ptr_prev = ptr;
        ptr = ptr->next;
    }

    ai = (area_info_t *) ib0->ptr.buffer;
    ai->next = tlsf->area_head;
    ai->end = lb0;
    tlsf->area_head = ai;
    free_ex(b0->ptr.buffer, mem_pool);
		
		
    return (b0->size & BLOCK_SIZE);
}

//
// get_used_size()
//
size_t get_used_size(void *mem_pool)
{
		//
		// pega a quantidade de memoria consumida 
		// da pool via acesso direto a info:
		//
    return ((tlsf_t *) mem_pool)->used_size;
}
//
// get_max_size()
//
size_t get_max_size(void *mem_pool)
{
	 //
	 // de forma similar ao get_size o max-size
	 // retorna o tamanho do maior bloco que da pra pegar
	 // da pool:
	 return ((tlsf_t *) mem_pool)->max_size;
}

//
// destroy_memory_pool()
//
void destroy_memory_pool(void *mem_pool)
{	
		//
		// para destruir a pool (em nossa aplicacao nao usado)
		// busca a mesma e apenas elimina a signature:
		//
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    tlsf->tlsf_signature = 0;
}

//
// malloc_ex()
//
void *malloc_ex(size_t size, void *mem_pool)
{
		//
		// implementacao baixo nivel do alloc:
		//
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    bhdr_t *b, *b2, *next_b;
    int32_t fl, sl;
    size_t tmp_size;

		//checagem e round de tamanho:
    size = (size < MIN_BLOCK_SIZE) ? MIN_BLOCK_SIZE : ROUNDUP_SIZE(size);

		//Busca os bit positions:
    MAPPING_SEARCH(&size, &fl, &sl);
	
		//Busca o bloco usando o good fit strategy
    b = FIND_SUITABLE_BLOCK(tlsf, &fl, &sl);
    
	  //Nao achou bloco? Retorna 0
		if (b == 0) return NULL;            

		//extrai o header
    EXTRACT_BLOCK_HDR(b, tlsf, fl, sl);
	
		//faz o tfsl apontar ao proximo blloco livre dessa buddy list
    next_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    tmp_size = (b->size & BLOCK_SIZE) - size;

		if (tmp_size >= sizeof(bhdr_t)) 
		{
				//
				// se o bloco for muito grande, faz split
			  //
        tmp_size -= BHDR_OVERHEAD;
        b2 = GET_NEXT_BLOCK(b->ptr.buffer, size);
        b2->size = tmp_size | FREE_BLOCK | PREV_USED;
        next_b->prev_hdr = b2;
        MAPPING_INSERT(tmp_size, &fl, &sl);
        INSERT_BLOCK(b2, tlsf, fl, sl);
        b->size = size | (b->size & PREV_STATE);
    } 
		else 
		{
        next_b->size &= (~PREV_FREE);
        b->size &= (~FREE_BLOCK);       
    }

		//
		// atualiza a estatistica da pool
		//
    TLSF_ADD_SIZE(tlsf, b);

		//bloco pronto pra uso:
    return (void *) b->ptr.buffer;
}

//
// free_ex()
//
void free_ex(void *ptr, void *mem_pool)
{
    tlsf_t *tlsf = (tlsf_t *) mem_pool;
    bhdr_t *b, *tmp_b;
    int32_t fl = 0, sl = 0;

		//bloco invalido? nao realiza acao
    if (!ptr) return;
	
	
    b = (bhdr_t *) ((uint8_t *) ptr - BHDR_OVERHEAD);
    b->size |= FREE_BLOCK;
    TLSF_REMOVE_SIZE(tlsf, b);

    b->ptr.free_ptr.prev = NULL;
    b->ptr.free_ptr.next = NULL;
	
		//
		// Pega o proximo bloco lire do buddy list
		//
    tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    
		//
		//se os blocos adjacentes estiverem livres
		//e fizerem parte do mesmo buddy list   
		//efetua a fusao destes em um unico  
		//bloco maior
		if (tmp_b->size & FREE_BLOCK) 
		{
        MAPPING_INSERT(tmp_b->size & BLOCK_SIZE, &fl, &sl);
        EXTRACT_BLOCK(tmp_b, tlsf, fl, sl);
        b->size += (tmp_b->size & BLOCK_SIZE) + BHDR_OVERHEAD;
    }
    if (b->size & PREV_FREE) 
		{
        tmp_b = b->prev_hdr;
        MAPPING_INSERT(tmp_b->size & BLOCK_SIZE, &fl, &sl);
        EXTRACT_BLOCK(tmp_b, tlsf, fl, sl);
        tmp_b->size += (b->size & BLOCK_SIZE) + BHDR_OVERHEAD;
        b = tmp_b;
    }
		
		//
		// Devolve o bloco ao buddylist
		//
    MAPPING_INSERT(b->size & BLOCK_SIZE, &fl, &sl);
    INSERT_BLOCK(b, tlsf, fl, sl);

		//
		// atualiza buddylist para proxima posicao livre nesse bloco
		//
    tmp_b = GET_NEXT_BLOCK(b->ptr.buffer, b->size & BLOCK_SIZE);
    tmp_b->size |= PREV_FREE;
    tmp_b->prev_hdr = b;
}

//
//Funcoes publicas:
// 
 
//
// HeapInit()
//
void HeapInit(uint8_t *heapp, uint32_t heapSize)
{
	int32_t size = 0;

	//
	// checa se o heap eh valido:
	//
	if(heapp == 0)
	{
		//
		// Mata o sistema de lock e nao inicializa
		//
		return;
	}
	
	//
	// Pede ao alocator pra preparar o heap:
	//
	size = init_memory_pool(heapSize, heapp);
	
	if(size == 0)
	{
		return;
	}
	
	//Inicializou corretamente, heap pronto para uso.
}


//
//	uMalloc()
//

void *uMalloc(uint32_t size)
{
	void *p = NULL;
	
	//
	// Limita o size do bloco:
	//
	if(size > 16384) size = 16000;
	
	//
	// Acessa o alocador em safe mode 
	//
	p = malloc_ex(size, mp);
	
	return(p);
}

//
// uFree()
//        
void uFree(void *p)
{
	//
	// checa consistencia do bloco:
	//
	if(p == NULL) return;
	free_ex(p, mp);
}

//
// uGetAvailable()
//
uint32_t uGetAvailable(void)
{
	if(mp != NULL)
	{
		return(get_max_size(mp) - get_used_size(mp));
	}
	else
 	{
		return 0;
	}		
}
