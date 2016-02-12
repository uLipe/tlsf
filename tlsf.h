//
// @file utils.h
// @brief Coletanea de support code util deve ser
//        colocada aqui.
//
//
#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>



// @fn ffs()
// @brief retorna o numero do bit onde aparece o 
//        set mais significativo
uint32_t uffs(uint32_t word);


//
// @fn fls()
// @brief retorna o numero do bit onde aparece o 
//        set menos significativo
uint32_t ufls(uint32_t word);

//
// @fn HeapInit()
// @brief Inicializa um Heap para usar como pool de memoria
//
void HeapInit(uint8_t *heapp,uint32_t heapSize);

//
//	@fn uMalloc()
//  @brief Aloca um bloco de memoria
//

void *uMalloc(uint32_t size);

//
// @fn uFree()
// @brief Destroi um bloco de memoria previamente
//        Alocado
void uFree(void *p);


//
// @fn uGetAvailable()
// @brief toma o espaco corrente do manager
//
uint32_t uGetAvailable(void);

#endif
