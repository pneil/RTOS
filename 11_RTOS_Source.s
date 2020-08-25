




	.def getPSP
	.def getR0
	.def getR1
	.def getR2
	.def getR3
	.def getR12
	.def getMSP
	.def getPC
	.def getLR
	.def getxPSR




.thumb
.text


getPSP:

   			MRS R0,PSP
   			BX LR

getR0:

   			BX LR

getR1:

   			MOV R0,R1
   			BX LR

getR2:

   			MOV R0,R2
   			BX LR

getR3:

   			MOV R0,R3
   			BX LR

getR12:

			MOV R0,R12
			BX LR

getLR:
			MOV R0,LR
			BX LR

getMSP:

   			MRS R0,MSP
   			BX LR

getPC:
			MOV R0,PC
			BX LR

getxPSR:
			MRS R0,APSR
			BX LR

.endm





