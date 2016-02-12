@
@
@  @file TraillingZeros.s
@  @brief Funcao para busca do bit menos significativo setado
@
@
@
			 .section .text
			 .thumb

			 .globl CntTrailZeros
			 .globl CntLeadZeros
@
@ CntTrailZeros()
@
CntTrailZeros:
			  rbit r0,r0			@ Change MSB with LSB position.
			  clz  r0,r0			@ use clz to find the "fake" MSB set
			  bx   lr				@ em 1 do novo word.

@
@ CntLeadZeros()
@
CntLeadZeros:
			  clz  r0,r0			@
			  bx   lr				@

			 .end
