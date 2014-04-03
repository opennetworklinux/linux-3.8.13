/*
 * MPC85xx cpu type detection
 *
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_PPC_CPU_H
#define __ASM_PPC_CPU_H

#define SVR_REV(svr)	((svr) & 0xFF)		/* SOC design resision */
#define SVR_MAJ(svr)	(((svr) >>  4) & 0xF)	/* Major revision field*/
#define SVR_MIN(svr)	(((svr) >>  0) & 0xF)	/* Minor revision field*/

/* Some parts define SVR[0:23] as the SOC version */
#define SVR_SOC_VER(svr) (((svr) >> 8) & 0xFFFFFF)	/* SOC Version fields */

#define IS_SVR_REV(svr, maj, min) \
	((SVR_MAJ(svr) == (maj)) && (SVR_MIN(svr) == (min)))

#define SVR_8533	0x803400
#define SVR_8533_E	0x803C00
#define SVR_8535	0x803701
#define SVR_8535_E	0x803F01
#define SVR_8536	0x803700
#define SVR_8536_E	0x803F00
#define SVR_8540	0x803000
#define SVR_8541	0x807200
#define SVR_8541_E	0x807A00
#define SVR_8543	0x803200
#define SVR_8543_E	0x803A00
#define SVR_8544	0x803401
#define SVR_8544_E	0x803C01
#define SVR_8545	0x803102
#define SVR_8545_E	0x803902
#define SVR_8547	0x803101
#define SVR_8547_E	0x803901
#define SVR_8548	0x803100
#define SVR_8548_E	0x803900
#define SVR_8555	0x807100
#define SVR_8555_E	0x807900
#define SVR_8560	0x807000
#define SVR_8567	0x807501
#define SVR_8567_E	0x807D01
#define SVR_8568	0x807500
#define SVR_8568_E	0x807D00
#define SVR_8569	0x808000
#define SVR_8569_E	0x808800
#define SVR_8572	0x80E000
#define SVR_8572_E	0x80E800
#define SVR_P1010	0x80f900
#define SVR_P1010_E	0x80F100
#define SVR_P2041	0x821001
#define SVR_P2041_E	0x821801
#define SVR_P3041	0x821103
#define SVR_P3041_E	0x821903
#define SVR_P5010	0x822100
#define SVR_P5010_E	0x822900
#define SVR_P5020	0x822000
#define SVR_P5020_E	0x822800
#define SVR_P5040	0x820400
#define SVR_P5040_E	0x820B00
#define SVR_T4240	0x824800
#define SVR_B4860	0x868800


static inline int fsl_svr_is(u32 svr)
{
	u32 id = SVR_SOC_VER(mfspr(SPRN_SVR));

	return (id == svr);
}

/* Check the SOC design version of this board */
static inline int fsl_svr_rev_is(u8 maj, u8 min)
{
	u32 rev = SVR_REV(mfspr(SPRN_SVR));
	u32 cmp = (maj << 4) | min;

	return (rev == cmp);
}

/* Return true if current SOC revision is prior to (maj, min)  */
static inline int fsl_svr_older_than(u8 maj, u8 min)
{
	u32 rev = SVR_REV(mfspr(SPRN_SVR));
	u32 cmp = (maj << 4) | min;

	return (rev < cmp);
}

#endif
