#include "tlsf.h"


#define BIT_WORD_SIZE 31


extern uint32_t CntLeadZeros(uint32_t word);
extern uint32_t CntTrailZeros(uint32_t word);

//
// ffs()
//
uint32_t ffs(uint32_t word)
{
	//
	// Adiciona o offset de delimitacao de word ao parametro
	// e usa o hardware do ARM para buscar o bit:
	//
	return( BIT_WORD_SIZE - CntLeadZeros(word));
}

//
// fls()
//
uint32_t fls(uint32_t word)
{
	//
	// Assim basta aplicar a mesma metodologia do ffs:
	//
	return(BIT_WORD_SIZE - CntTrailZeros(word));
}

