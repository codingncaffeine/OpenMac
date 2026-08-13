// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/***************************************************************************

    am29ops.h
    Am29000 instructions

***************************************************************************/

/***************************************************************************
    DEFINES
***************************************************************************/

#define IFLAG_ILLEGAL               (1 << 0)
#define IFLAG_SUPERVISOR_ONLY       (1 << 1)
#define IFLAG_RA_PRESENT            (1 << 2)
#define IFLAG_RB_PRESENT            (1 << 3)
#define IFLAG_RC_PRESENT            (1 << 4)

#define IFLAG_SPR_ACCESS            (1 << 6)
#define IFLAG_MEMORY_ACCESS         (1 << 8)
#define IFLAG_CONTROL               (1 << 9)


#define GET_RA_VAL                  (m_r[RA])
#define GET_RB_VAL                  (m_r[RB])

#define RA                          (get_abs_reg(m_exec_ir >>  8, m_ipa))
#define RB                          (get_abs_reg(m_exec_ir >>  0, m_ipb))
#define RC                          (get_abs_reg(m_exec_ir >> 16, m_ipc))

#define INST_SA                     ((m_exec_ir >> 8) & 0xff)
#define INST_VN                     ((m_exec_ir >> 16) & 0xff)
#define INST_M_BIT                  (m_exec_ir & (1 << 24))
#define INST_CE_BIT                 (m_exec_ir & (1 << 23))
#define INST_AS_BIT                 (m_exec_ir & (1 << 22))
#define INST_PA_BIT                 (m_exec_ir & (1 << 21))
#define INST_SB_BIT                 (m_exec_ir & (1 << 20))
#define INST_UA_BIT                 (m_exec_ir & (1 << 19))
#define INST_OPT_MASK               (7)
#define INST_OPT_SHIFT              (16)
#define INST_OPT_FIELD              (((m_exec_ir) >> INST_OPT_SHIFT) & INST_OPT_MASK)
#define INST_CNTL_MASK              (0x7f)
#define INST_CNTL_SHIFT             (16)

#define I8                          (m_exec_ir & 0xff)
#define I16                         (((m_exec_ir >> 8) & 0xff00) | (m_exec_ir & 0xff))
#define I16_ZEX                     ((uint32_t)(I16))
#define I16_SEX                     ((int32_t)(int16_t)I16)
#define I16_OEX                     (0xffff0000 | I16)

#define JMP_ZEX                     (I16 << 2)
#define JMP_SEX                     ((int32_t)(int16_t)(((m_exec_ir >> 8) & 0xff00) | (m_exec_ir & 0xff)) << 2)

#define BOOLEAN_MASK                (1 << 31)
#define BOOLEAN_TRUE                (1 << 31)
#define BOOLEAN_FALSE               (0)

#define UNHANDLED_OP                fatalerror("Am29000: Unhandled inst %s at %x\n", __FUNCTION__, m_exec_pc);


/***************************************************************************
    ALU FLAG CALCULATION
***************************************************************************/

#define SET_ALU_Z(r)            m_alu &= ~ALU_Z; \
								m_alu |= (r == 0) << ALU_Z_SHIFT;

#define SET_ALU_N(r)            m_alu &= ~ALU_N; \
								m_alu |= ((uint32_t)r & 0x80000000) >> (31 - ALU_N_SHIFT);

#define CALC_C_ADD(r, a)        ((uint32_t)(r) < (uint32_t)(a))

#define SET_ALU_C_ADD(r, a)     m_alu &= ~ALU_C; \
								m_alu |= CALC_C_ADD(r, a) << ALU_C_SHIFT;

#define CALC_C_SUB(a, b)        (!((uint32_t)(a) < (uint32_t)(b)))

#define SET_ALU_C_SUB(a, b)     m_alu &= ~ALU_C; \
								m_alu |= CALC_C_SUB(a, b) << ALU_C_SHIFT;

#define SET_ALU_V_ADD(r, a, b)  m_alu &= ~ALU_V; \
								m_alu |= (((int32_t)(~((a) ^ (b)) & ((a) ^ (r))) < 0)) << ALU_V_SHIFT;

#define SET_ALU_V_SUB(r, a, b)  m_alu &= ~ALU_V; \
								m_alu |= ((int32_t)(((a) ^ (b)) & ((a) ^ (r))) < 0) << ALU_V_SHIFT;

#define GET_CARRY               ((m_alu >> ALU_C_SHIFT) & 1)



uint32_t Am29000::read_spr(uint32_t idx)
{
	uint32_t val = 0;

	switch (idx)
	{
		case SprVab:   val = m_vab;     break;
		case SprOps:   val = m_ops;     break;
		case SprCps:   val = m_cps;     break;
		case SprCfg:   val = m_cfg;     break;
		case SprCha:   val = m_cha;     break;
		case SprChd:   val = m_chd;     break;
		case SprChc:   val = m_chc;     break;
		case SprRbp:   val = m_rbp;     break;
		case SprTmc:   val = m_tmc & TCV_MASK; break;
		case SprTmr:   val = m_tmr;     break;
		case SprPc0:   val = m_pc0;     break;
		case SprPc1:   val = m_pc1;     break;
		case SprPc2:   val = m_pc2;     break;
		case SprMmu:   val = m_mmu;     break;
		case SprLru:   val = m_lru;     break;
		case SprIpc:   val = m_ipc;     break;
		case SprIpa:   val = m_ipa;     break;
		case SprIpb:   val = m_ipb;     break;
		case SprQ:     val = m_q;       break;
		case SprAlu:   val = m_alu;     break;
		case SprBp:    val = GET_ALU_BP;       break;
		case SprFc:    val = GET_ALU_FC;       break;
		case SprCr:    val = GET_CHC_CR;       break;
		case SprFpe:   val = m_fpe;     break;
		case SprInte:  val = m_inte;    break;
		case SprFps:   val = m_fps;     break;
		default:
			logerror("Unknown SPR read (%d)\n", idx);
	}

	return val;
}


void Am29000::write_spr(uint32_t idx, uint32_t val)
{
	switch (idx)
	{
		case SprVab:   m_vab = val & (VAB_MASK << VAB_SHIFT);
						break;
		case SprOps:   m_ops = val & (CPS_CA | CPS_IP | CPS_TE | CPS_TP | CPS_TU | CPS_FZ | CPS_LK | CPS_RE |
						CPS_WM | CPS_PD | CPS_PI | CPS_SM | (CPS_IM_MASK << CPS_IM_SHIFT) | CPS_DI | CPS_DA);
						break;
		case SprCps:   m_cps = val & (CPS_CA | CPS_IP | CPS_TE | CPS_TP | CPS_TU | CPS_FZ | CPS_LK | CPS_RE |
						CPS_WM | CPS_PD | CPS_PI | CPS_SM | (CPS_IM_MASK << CPS_IM_SHIFT) | CPS_DI | CPS_DA);
						break;
		case SprCfg:   m_cfg = val & (CFG_DW | CFG_VF | CFG_RV | CFG_BO | CFG_CP | CFG_CD);
						m_cfg |= PROCESSOR_REL_FIELD << CFG_PRL_SHIFT;
						break;
		case SprCha:   m_cha = val;
						break;
		case SprChd:   m_chd = val;
						break;
		case SprChc:   m_chc = val;
						break;
		case SprRbp:   m_rbp = val & RBP_MASK;
						break;
		case SprTmc:   m_tmc = val & TCV_MASK;
						break;
		case SprTmr:   m_tmr = val & (TMR_OV | TMR_IN | TMR_IE | TMR_TRV_MASK);
						break;
		case SprPc0:   m_pc0 = val & PC_MASK;
						break;
		case SprPc1:   m_pc1 = val & PC_MASK;
						break;
		case SprPc2:   m_pc2 = val & PC_MASK;
						break;
		case SprMmu:   m_mmu = val & ((MMU_PS_MASK << MMU_PS_SHIFT) | MMU_PID_MASK);
						break;
		case SprLru:   m_lru = val & (LRU_MASK << LRU_SHIFT);
						break;
		case SprIpc:   m_ipc = val;// & IPX_MASK;
						break;
		case SprIpa:   m_ipa = val;// & IPX_MASK;
						break;
		case SprIpb:   m_ipb = val;// & IPX_MASK;
						break;
		case SprQ:     m_q = val;
						break;
		case SprAlu:   m_alu = val & (ALU_DF | ALU_V | ALU_N | ALU_Z | ALU_C | (ALU_BP_MASK << ALU_BP_SHIFT) | (ALU_FC_MASK << ALU_FC_SHIFT));
						break;
		case SprBp:    m_alu &= ~(ALU_BP_MASK << ALU_BP_SHIFT);
						m_alu |= (val & ALU_BP_MASK) << ALU_BP_SHIFT;
						break;
		case SprFc:    m_alu &= ~(ALU_FC_MASK << ALU_FC_SHIFT);
						m_alu |= (val & ALU_FC_MASK) << ALU_FC_SHIFT;
						break;
		case SprCr:    m_chc &= ~(CHC_CR_MASK << CHC_CR_SHIFT);
						m_chc |= (val & CHC_CR_MASK) << CHC_CR_SHIFT;
						break;
//      case SprFpe:
//      case SprInte:
//      case SprFps:
		default: logerror("Unhandled SPR write (%d)\n", idx);
	}
}


/***************************************************************************
    INSTRUCTIONS
***************************************************************************/

/***************************************************************************
    INTEGER ARITHMETIC
***************************************************************************/

void Am29000::ADD()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a + b;

	if (!FREEZE_MODE)
	{
		SET_ALU_V_ADD(r, a, b);
		SET_ALU_Z(r);
		SET_ALU_N(r);
		SET_ALU_C_ADD(r, a);
	}

	m_r[RC] = r;
}

void Am29000::ADDS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = addArithmetic(a, b, 0);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.signedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::ADDU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = addArithmetic(a, b, 0);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::ADDC()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = addArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	m_r[RC] = result.value;
}

void Am29000::ADDCS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	const ArithmeticResult result = addArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.signedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::ADDCU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = addArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUB()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r = a - b;

	if (!FREEZE_MODE)
	{
		SET_ALU_V_SUB(r, a, b);
		SET_ALU_Z(r);
		SET_ALU_N(r);
		SET_ALU_C_SUB(a, b);
	}

	// TODO: Trap on unsigned overflow
	m_r[RC] = r;
}

void Am29000::SUBS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r = a - b;

	if (!FREEZE_MODE)
	{
		SET_ALU_V_SUB(r, a, b);
		SET_ALU_Z(r);
		SET_ALU_N(r);
		SET_ALU_C_SUB(a, b);
	}

	if ((int32_t)(((a) ^ (b)) & ((a) ^ (r))) < 0)
		SIGNAL_EXCEPTION(ExceptionOutOfRange);

	m_r[RC] = r;
}

void Am29000::SUBU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(a, b, 1);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBC()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	m_r[RC] = result.value;
}

void Am29000::SUBCS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.signedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBCU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(a, b, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBR()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r = b - a;

	if (!FREEZE_MODE)
	{
		SET_ALU_V_SUB(r, a, b);
		SET_ALU_Z(r);
		SET_ALU_N(r);
		SET_ALU_C_SUB(a, b);
	}

	m_r[RC] = r;
}

void Am29000::SUBRS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(b, a, 1);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.signedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBRU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(b, a, 1);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBRC()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(b, a, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	m_r[RC] = result.value;
}

void Am29000::SUBRCS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(b, a, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.signedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::SUBRCU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	const ArithmeticResult result = subtractArithmetic(b, a, GET_CARRY);
	if (!FREEZE_MODE) setArithmeticFlags(m_alu, result);
	if (result.unsignedOverflow) SIGNAL_EXCEPTION(ExceptionOutOfRange);
	m_r[RC] = result.value;
}

void Am29000::MULTIPLU()
{
	UNHANDLED_OP;
}

void Am29000::MULTIPLY()
{
	UNHANDLED_OP;
}

void Am29000::MUL()
{
	/* TODO: Zero/Neg flags ? */
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r;
	uint32_t sign;

	if (m_q & 1)
	{
		r = a + b;
		sign = (r >> 31) ^ (((int32_t)(~((a) ^ (b)) & ((a) ^ (r))) < 0));
	}
	else
	{
		r = b;
		sign = b >> 31;
	}

	// Shift combined value right one bit position, entering sign into bit 31 of destination
	m_r[RC] = (r >> 1) | (sign << 31);
	m_q = (m_q >> 1) | (r << 31);
}

void Am29000::MULL()
{
	/* TODO: Zero/Neg flags ? */
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r;
	uint32_t sign;

	if (m_q & 1)
	{
		r = b - a;
		sign = (r >> 31) ^ ((int32_t)(((a) ^ (b)) & ((a) ^ (r))) < 0);
	}
	else
	{
		r = b;
		sign = b >> 31;
	}

	// Shift combined value right one bit position, entering sign into bit 31 of destination
	m_r[RC] = (r >> 1) | (sign << 31);
	m_q = (m_q >> 1) | (r << 31);
}

void Am29000::MULU()
{
	/* TODO: Zero/Neg flags ? */
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r;
	uint32_t c;

	if (m_q & 1)
	{
		r = a + b;
		c = (uint32_t)(r) < (uint32_t)(a);
	}
	else
	{
		r = b;
		c = 0;
	}

	// Shift combined value right one bit position, entering carry into bit 31 of destination
	m_r[RC] = (r >> 1) | (c << 31);
	m_q = (m_q >> 1) | (r << 31);
}

void Am29000::DIVIDE()
{
	m_ipa = RA << IPX_SHIFT;
	m_ipb = RB << IPX_SHIFT;
	m_ipc = RC << IPX_SHIFT;

	SIGNAL_EXCEPTION(ExceptionDivide);
}

void Am29000::DIVIDU()
{
	UNHANDLED_OP;
}

void Am29000::DIV0()
{
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;

	if (!FREEZE_MODE)
	{
		m_alu |= ALU_DF;
		SET_ALU_N(b);
	}

	// Shift combined value left one bit position, zeroing bit 0 of Q
	m_r[RC] = (b << 1) | (m_q >> 31);
	m_q <<= 1;
}

void Am29000::DIV()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t c;
	uint32_t r;
	uint32_t df;

	if (m_alu & ALU_DF)
	{
		r = a - b;
		c = !((uint32_t)(a) < (uint32_t)(b));
	}
	else
	{
		r = a + b;
		c = (uint32_t)(r) < (uint32_t)(a);
	}


	df = (~(c ^ (m_alu >> ALU_DF_SHIFT) ^ (m_alu >> ALU_N_SHIFT)) & 1);

	if (!FREEZE_MODE)
	{
		m_alu &= ~ALU_DF;
		m_alu |= df << ALU_DF_SHIFT;
		SET_ALU_N(r);
	}

	// Shift combined value left one bit position, entering DF into bit 0 of Q
	m_r[RC] = (r << 1) | (m_q >> 31);
	m_q = (m_q << 1) | df;
}

void Am29000::DIVL()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t c;
	uint32_t r;
	uint32_t df;

	if (m_alu & ALU_DF)
	{
		r = a - b;
		c = !((uint32_t)(a) < (uint32_t)(b));
	}
	else
	{
		r = a + b;
		c = (uint32_t)(r) < (uint32_t)(a);
	}

	df = (~(c ^ (m_alu >> ALU_DF_SHIFT) ^ (m_alu >> ALU_N_SHIFT)) & 1);

	if (!FREEZE_MODE)
	{
		m_alu &= ~ALU_DF;
		m_alu |= df << ALU_DF_SHIFT;
		SET_ALU_N(r);
	}

	m_q = (m_q << 1) | df;
	m_r[RC] = r;
}

void Am29000::DIVREM()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;

	if (m_alu & ALU_DF)
		m_r[RC] = a;
	else
		m_r[RC] = a + b;
}


/***************************************************************************
    COMPARE
***************************************************************************/

void Am29000::CPEQ()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a == b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPNEQ()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a != b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPLT()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (int32_t)a < (int32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPLTU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (uint32_t)a < (uint32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPLE()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (int32_t)a <= (int32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPLEU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (uint32_t)a <= (uint32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPGT()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (int32_t)a > (int32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPGTU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (uint32_t)a > (uint32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPGE()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (int32_t)a >= (int32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPGEU()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = (uint32_t)a >= (uint32_t)b ? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::CPBYTE()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8 : GET_RB_VAL;
	uint32_t r =
			((a & 0xff000000) == (b & 0xff000000)) ||
			((a & 0x00ff0000) == (b & 0x00ff0000)) ||
			((a & 0x0000ff00) == (b & 0x0000ff00)) ||
			((a & 0x000000ff) == (b & 0x000000ff))
			? BOOLEAN_TRUE : BOOLEAN_FALSE;

	m_r[RC] = r;
}

void Am29000::ASEQ()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!(GET_RA_VAL == GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASNEQ()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!(GET_RA_VAL != GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASLT()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((int32_t)GET_RA_VAL < (int32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASLTU()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((uint32_t)GET_RA_VAL < (uint32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASLE()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((int32_t)GET_RA_VAL <= (int32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASLEU()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((uint32_t)GET_RA_VAL <= (uint32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASGT()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((int32_t)GET_RA_VAL > (int32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASGTU()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((uint32_t)GET_RA_VAL > (uint32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASGE()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((int32_t)GET_RA_VAL >= (int32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}

void Am29000::ASGEU()
{
	if (USER_MODE && INST_VN < 64)
		SIGNAL_EXCEPTION(ExceptionProtectionViolation);
	else if (!((uint32_t)GET_RA_VAL >= (uint32_t)GET_RB_VAL))
		SIGNAL_EXCEPTION(INST_VN);
}


/***************************************************************************
    LOGICAL
***************************************************************************/

void Am29000::AND()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a & b;

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::ANDN()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a & ~b;

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::NAND()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = ~(a & b);

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::OR()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a | b;

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::NOR()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = ~(a | b);

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::XOR()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = a ^ b;

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}

void Am29000::XNOR()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r = ~(a ^ b);

	if (!FREEZE_MODE)
	{
		SET_ALU_Z(r);
		SET_ALU_N(r);
	}

	m_r[RC] = r;
}


/***************************************************************************
    SHIFT
***************************************************************************/

void Am29000::SLL()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = (INST_M_BIT ? I8: GET_RB_VAL) & 0x1f;
	uint32_t r = a << b;

	m_r[RC] = r;
}

void Am29000::SRL()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = (INST_M_BIT ? I8: GET_RB_VAL) & 0x1f;
	uint32_t r = a >> b;

	m_r[RC] = r;
}

void Am29000::SRA()
{
	int32_t a = GET_RA_VAL;
	uint32_t b = (INST_M_BIT ? I8: GET_RB_VAL) & 0x1f;
	uint32_t r = a >> b;

	m_r[RC] = r;
}

void Am29000::EXTRACT()
{
	int32_t a = GET_RA_VAL;
	uint32_t b = (INST_M_BIT ? I8: GET_RB_VAL);
	uint64_t r;

	r = (((uint64_t)a << 32) | b) << GET_ALU_FC;

	m_r[RC] = r >> 32;
}


/***************************************************************************
    DATA MOVEMENT
***************************************************************************/

void Am29000::LOAD()
{
	uint32_t addr = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t r;
	const uint32_t target = RA;
	const uint32_t channelControl =
		((m_exec_ir >> 16) & CHC_CE_CNTL_MASK) << CHC_CE_CNTL_SHIFT |
		CHC_LS | target << CHC_TR_SHIFT | CHC_CV;

	if (INST_CE_BIT)
	{
		logerror("Am29000: Attempting a co-processor LOAD!\n");
		r = 0;
	}
	else
	{
		if (INST_PA_BIT && USER_MODE)
		{
			SIGNAL_EXCEPTION(ExceptionProtectionViolation);
			return;
		}
		if (trapsUnalignedDataAccess(addr, INST_OPT_FIELD,
			INST_AS_BIT == 0))
		{
			if (!FREEZE_MODE)
			{
				m_chc = channelControl;
				m_cha = addr;
				m_chd = 0;
			}
			SIGNAL_EXCEPTION(ExceptionUnalignedAccess);
			return;
		}
		uint32_t physical = addr;
		const bool userAccess = USER_MODE || INST_UA_BIT;
		const bool inputOutput = INST_AS_BIT != 0;
		// Only instruction/data-memory addresses pass through the MMU.  AS=1
		// selects the independent input/output address space and the numeric
		// address is presented directly to the external bus.
		if (!inputOutput && !INST_PA_BIT && !(m_cps & CPS_PD) &&
			!translateAddress(addr, Access::Load, userAccess, physical))
			return;
		// Rev-C and later parts use SB as signed-byte/half-word selection when
		// CFG.DW enables the external-width hardware.  GC QuickDraw relies on
		// this for signed 16-bit Rect/Point coordinates in its command stream.
		r = read_data_value(physical, INST_OPT_FIELD,
			(m_cfg & CFG_DW) != 0 && INST_SB_BIT != 0, inputOutput);
	}

//  if (opt & 2)
//      logerror("Am29000: Half word LOAD\n");

	if (!FREEZE_MODE)
	{
		m_chc = channelControl;
		m_cha = addr;
		m_chd = r;

		if (!(m_cfg & CFG_DW) && INST_SB_BIT)
		{
			SET_ALU_BP(addr);
		}
	}

	m_r[target] = r;

	if (m_cfg & CFG_DW)
	{
		logerror("DW ON A STORE");
	}
}

void Am29000::LOADL()
{
	UNHANDLED_OP;
}

void Am29000::LOADSET()
{
	UNHANDLED_OP;
}

void Am29000::LOADM()
{
	uint32_t addr = INST_M_BIT ? I8: GET_RB_VAL;
	const uint32_t target = RA;
	const uint32_t channelControl =
		(m_chc & (CHC_CR_MASK << CHC_CR_SHIFT)) |
		((m_exec_ir >> 16) & CHC_CE_CNTL_MASK) << CHC_CE_CNTL_SHIFT |
		CHC_LS | CHC_ML | target << CHC_TR_SHIFT | CHC_CV;

	uint32_t r;
	if (INST_CE_BIT)
	{
		logerror("Am29000: Attempting a co-processor LOAD!\n");
		r = 0;
	}
	else
	{
		if (INST_PA_BIT && USER_MODE)
		{
			SIGNAL_EXCEPTION(ExceptionProtectionViolation);
			return;
		}
		if (trapsUnalignedDataAccess(addr, INST_OPT_FIELD,
			INST_AS_BIT == 0))
		{
			if (!FREEZE_MODE)
			{
				m_chc = channelControl;
				m_cha = addr;
				m_chd = 0;
			}
			SIGNAL_EXCEPTION(ExceptionUnalignedAccess);
			return;
		}
		// LOADM performs exactly CR+1 transfers in the loop below.  The
		// inherited core used to issue an extra throwaway read here, which
		// is observable by Dolphin's MFB burst-driven instruction-cache fill.
		r = 0;
	}

	if (!FREEZE_MODE)
	{
		m_chc = channelControl;
		m_cha = addr;
		m_chd = r; // ?????

		if (!(m_cfg & CFG_DW) && INST_SB_BIT)
		{
			SET_ALU_BP(addr);
		}
	}

	r = target;

	for (int32_t cnt = 0; cnt <= GET_CHC_CR; ++cnt)
	{
		uint32_t physical = addr;
		const bool userAccess = USER_MODE || INST_UA_BIT;
		const bool inputOutput = INST_AS_BIT != 0;
		if (!inputOutput && !INST_PA_BIT && !(m_cps & CPS_PD) &&
			!translateAddress(addr, Access::Load, userAccess, physical))
			return;
		m_r[r] = read_data_value(physical, INST_OPT_FIELD,
			(m_cfg & CFG_DW) != 0 && INST_SB_BIT != 0, inputOutput);

//          SET_CHC_CR(cnt - 1);
		addr += 4;

		if (++r == 256)
		{
			r = 128;
		}
	}
}

void Am29000::STORE()
{
	uint32_t addr = INST_M_BIT ? I8: GET_RB_VAL;
	const uint32_t source = RA;
	const uint32_t value = m_r[source];
	const uint32_t channelControl =
		((m_exec_ir >> 16) & CHC_CE_CNTL_MASK) << CHC_CE_CNTL_SHIFT |
		source << CHC_TR_SHIFT | CHC_CV;

	if (INST_CE_BIT)
	{
		logerror("Am29000: Attempting a co-processor LOAD!\n");
	}
	else
	{
		if (INST_PA_BIT && USER_MODE)
		{
			SIGNAL_EXCEPTION(ExceptionProtectionViolation);
			return;
		}
		if (trapsUnalignedDataAccess(addr, INST_OPT_FIELD,
			INST_AS_BIT == 0))
		{
			if (!FREEZE_MODE)
			{
				m_chc = channelControl;
				m_cha = addr;
				m_chd = value;
			}
			SIGNAL_EXCEPTION(ExceptionUnalignedAccess);
			return;
		}
	}
	uint32_t physical = addr;
	const bool userAccess = USER_MODE || INST_UA_BIT;
	const bool inputOutput = INST_AS_BIT != 0;
	if (!inputOutput && !INST_PA_BIT && !(m_cps & CPS_PD) &&
		!translateAddress(addr, Access::Store, userAccess, physical))
		return;
	write_data_value(physical, value, INST_OPT_FIELD, inputOutput);

	if (!FREEZE_MODE)
	{
		m_chc = channelControl;
		m_cha = addr;
		m_chd = value;

		if (!(m_cfg & CFG_DW) && INST_SB_BIT)
		{
			SET_ALU_BP(addr);
		}
	}

	if (m_cfg & CFG_DW)
	{
		logerror("DW ON A STORE");
	}
}

void Am29000::STOREL()
{
	UNHANDLED_OP;
}

void Am29000::STOREM()
{
	uint32_t addr = INST_M_BIT ? I8: GET_RB_VAL;
	const uint32_t source = RA;
	const uint32_t channelControl =
		(m_chc & (CHC_CR_MASK << CHC_CR_SHIFT)) |
		((m_exec_ir >> 16) & CHC_CE_CNTL_MASK) << CHC_CE_CNTL_SHIFT |
		CHC_ML | source << CHC_TR_SHIFT | CHC_CV;

	uint32_t r;
	if (INST_CE_BIT)
	{
		logerror("Am29000: Attempting a co-processor LOAD!\n");
		r = 0;
	}
	else
	{
		if (INST_PA_BIT && USER_MODE)
		{
			SIGNAL_EXCEPTION(ExceptionProtectionViolation);
			return;
		}
		if (trapsUnalignedDataAccess(addr, INST_OPT_FIELD,
			INST_AS_BIT == 0))
		{
			if (!FREEZE_MODE)
			{
				m_chc = channelControl;
				m_cha = addr;
				m_chd = m_r[source];
			}
			SIGNAL_EXCEPTION(ExceptionUnalignedAccess);
			return;
		}
	}

	if (!FREEZE_MODE)
	{
		m_chc = channelControl;
		m_cha = addr;
		m_chd = m_r[source];

		if (!(m_cfg & CFG_DW) && INST_SB_BIT)
		{
			SET_ALU_BP(addr);
		}
	}

	r = source;

	for (int32_t cnt = 0; cnt <= GET_CHC_CR; ++cnt)
	{
		uint32_t physical = addr;
		const bool userAccess = USER_MODE || INST_UA_BIT;
		const bool inputOutput = INST_AS_BIT != 0;
		if (!inputOutput && !INST_PA_BIT && !(m_cps & CPS_PD) &&
			!translateAddress(addr, Access::Store, userAccess, physical))
			return;
		write_data_value(physical, m_r[r], INST_OPT_FIELD, inputOutput);

		addr += 4;

		if (++r == 256)
		{
			r = 128;
		}
	}
}

void Am29000::EXBYTE()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t bp = GET_ALU_BP;
	uint8_t srcbyte;
	uint32_t r;

	if (m_cfg & CFG_BO)
		srcbyte = a >> 8 * bp;
	else
		srcbyte = a >> (8 * (3 - bp));

	r = (b & 0xffffff00) | srcbyte;

	m_r[RC] = r;
}

void Am29000::EXHW()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t wp = ((m_alu >> ALU_BP_SHIFT) & ALU_BP_MASK) >> 1;
	uint16_t srcword;
	uint32_t r;

	if (m_cfg & CFG_BO)
		srcword = a >> 16 * wp;
	else
		srcword = a >> (16 * (1 - wp));

	r = (b & 0xffff0000) | srcword;

	m_r[RC] = r;
}

void Am29000::EXHWS()
{
	uint32_t a = GET_RA_VAL;
	uint32_t wp = ((m_alu >> ALU_BP_SHIFT) & ALU_BP_MASK) >> 1;
	uint16_t srcword;
	uint32_t r;

	if (m_cfg & CFG_BO)
		srcword = a >> 16 * wp;
	else
		srcword = a >> (16 * (1 - wp));

	r = (int32_t)(int16_t)srcword;

	m_r[RC] = r;
}

void Am29000::INBYTE()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t bp = GET_ALU_BP;
	uint8_t shift = (m_cfg & CFG_BO) ? 8 * bp : (8 * (3 - bp));
	uint32_t r;

	r = (a & ~(0xff << shift)) | ((b & 0xff) << shift);

	m_r[RC] = r;
}

void Am29000::INHW()
{
	uint32_t a = GET_RA_VAL;
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;
	uint32_t wp = ((m_alu >> ALU_BP_SHIFT) & ALU_BP_MASK) >> 1;
	uint32_t shift = (m_cfg & CFG_BO) ? 16 * wp : (16 * (1 - wp));
	uint32_t r;

	r = (a & ~(0xffff << shift)) | ((b & 0xffff) << shift);

	m_r[RC] = r;
}

void Am29000::MFSR()
{
	m_r[RC] = read_spr(INST_SA);
}

void Am29000::MFTLB()
{
	m_r[RC] = m_tlb[GET_RA_VAL & 0x7f];
}

void Am29000::MTSR()
{
	write_spr(INST_SA, GET_RB_VAL);
}

void Am29000::MTSRIM()
{
	write_spr(INST_SA, I16_ZEX);
}

void Am29000::MTTLB()
{
	m_tlb[GET_RA_VAL & 0x7f] = GET_RB_VAL;
}


/***************************************************************************
    CONSTANT
***************************************************************************/

void Am29000::CONST()
{
	m_r[RA] = I16_ZEX;
}

void Am29000::CONSTH()
{
	m_r[RA] = (I16 << 16) | GET_RA_VAL;
}

void Am29000::CONSTN()
{
	m_r[RA] = I16_OEX;
}


/***************************************************************************
    BRANCH INSTRUCTIONS
***************************************************************************/

void Am29000::CALL()
{
	uint32_t ret = m_next_pc;

	if (INST_M_BIT)
		m_next_pc = JMP_ZEX;
	else
		m_next_pc = m_exec_pc + JMP_SEX;

	m_r[RA] = ret;
	m_next_pl_flags |= PFLAG_JUMP;
}

void Am29000::CALLI()
{
	uint32_t ret = m_next_pc;
	m_next_pc = GET_RB_VAL;
	m_r[RA] = ret;
	m_next_pl_flags |= PFLAG_JUMP;
}

void Am29000::JMP()
{
	if (INST_M_BIT)
		m_next_pc = JMP_ZEX;
	else
		m_next_pc = m_exec_pc + JMP_SEX;

	m_next_pl_flags |= PFLAG_JUMP;
}

void Am29000::JMPI()
{
	m_next_pc = GET_RB_VAL;

	m_next_pl_flags |= PFLAG_JUMP;
}

void Am29000::JMPT()
{
	if ((GET_RA_VAL & BOOLEAN_MASK) == BOOLEAN_TRUE)
	{
		if (INST_M_BIT)
			m_next_pc = JMP_ZEX;
		else
			m_next_pc = m_exec_pc + JMP_SEX;

		m_next_pl_flags |= PFLAG_JUMP;
	}
}

void Am29000::JMPTI()
{
	if ((GET_RA_VAL & BOOLEAN_MASK) == BOOLEAN_TRUE)
	{
		m_next_pc = GET_RB_VAL;
		m_next_pl_flags |= PFLAG_JUMP;
	}
}

void Am29000::JMPF()
{
	if ((GET_RA_VAL & BOOLEAN_MASK) == BOOLEAN_FALSE)
	{
		if (INST_M_BIT)
			m_next_pc = JMP_ZEX;
		else
			m_next_pc = m_exec_pc + JMP_SEX;

		m_next_pl_flags |= PFLAG_JUMP;
	}
}

void Am29000::JMPFI()
{
	if ((GET_RA_VAL & BOOLEAN_MASK) == BOOLEAN_FALSE)
	{
		m_next_pc = GET_RB_VAL;
		m_next_pl_flags |= PFLAG_JUMP;
	}
}

void Am29000::JMPFDEC()
{
	uint32_t a = GET_RA_VAL;

	if ((a & BOOLEAN_MASK) == BOOLEAN_FALSE)
	{
		if (INST_M_BIT)
			m_next_pc = JMP_ZEX;
		else
			m_next_pc = m_exec_pc + JMP_SEX;

		m_next_pl_flags |= PFLAG_JUMP;
	}

	m_r[RA] = a - 1;
}


/***************************************************************************
    MISCELLANEOUS INSTRUCTIONS
***************************************************************************/

void Am29000::CLZ()
{
	uint32_t b = INST_M_BIT ? I8: GET_RB_VAL;

	m_r[RC] = std::countl_zero(b);
}

void Am29000::SETIP()
{
	m_ipa = RA << IPX_SHIFT;
	m_ipb = RB << IPX_SHIFT;
	m_ipc = RC << IPX_SHIFT;
}

void Am29000::EMULATE()
{
	UNHANDLED_OP;
}

void Am29000::INV()
{
	/* Nothing to do yet */
}

void Am29000::IRET()
{
	m_iret_pc = m_pc0;
	m_next_pc = m_pc1;
	m_cps = m_ops;
	m_next_pl_flags = PFLAG_IRET;
}

void Am29000::IRETINV()
{
	// Architecturally this is IRET plus invalidation of the branch-target
	// cache.  This portable core has no decoded-instruction cache, so the
	// invalidation has no observable state and the return sequence is exactly
	// the one implemented by IRET.
	IRET();
}

void Am29000::HALT()
{
	UNHANDLED_OP;
}


void Am29000::ILLEGAL()
{
	SIGNAL_EXCEPTION(ExceptionIllegalOpcode);
}



/***************************************************************************
    UNHANDLED
***************************************************************************/

void Am29000::CONVERT()
{
	UNHANDLED_OP;
}

void Am29000::SQRT()
{
	UNHANDLED_OP;
}

void Am29000::CLASS()
{
	UNHANDLED_OP;
}

void Am29000::MULTM()
{
	UNHANDLED_OP;
}

void Am29000::MULTMU()
{
	UNHANDLED_OP;
}


const std::array<Am29000::OpcodeInfo, 256> Am29000::opTable_ =
{{
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::CONSTN,   IFLAG_RA_PRESENT                                                   },
	{ &Am29000::CONSTH,   IFLAG_RA_PRESENT                                                   },
	{ &Am29000::CONST,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::MTSRIM,   0                                                                  },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::LOADL,    0                                                                  },
	{ &Am29000::LOADL,    0                                                                  },
	{ &Am29000::CLZ,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::CLZ,      IFLAG_RC_PRESENT                                                   },
	{ &Am29000::EXBYTE,   IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::EXBYTE,   IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::INBYTE,   IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::INBYTE,   IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::STOREL,   0                                                                  },
	{ &Am29000::STOREL,   0                                                                  },
	{ &Am29000::ADDS,     0                                                                  },
	{ &Am29000::ADDS,     0                                                                  },
	{ &Am29000::ADDU,     0                                                                  },
	{ &Am29000::ADDU,     0                                                                  },
	{ &Am29000::ADD,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::ADD,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::LOAD,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::LOAD,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ADDCS,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::ADDCS,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ADDCU,    0                                                                  },
	{ &Am29000::ADDCU,    0                                                                  },
	{ &Am29000::ADDC,     0                                                                  },
	{ &Am29000::ADDC,     0                                                                  },
	{ &Am29000::STORE,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::STORE,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::SUBS,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SUBS,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::SUBU,     0                                                                  },
	{ &Am29000::SUBU,     0                                                                  },
	{ &Am29000::SUB,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SUB,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::LOADSET,  0                                                                  },
	{ &Am29000::LOADSET,  0                                                                  },
	{ &Am29000::SUBCS,    0                                                                  },
	{ &Am29000::SUBCS,    0                                                                  },
	{ &Am29000::SUBCU,    0                                                                  },
	{ &Am29000::SUBCU,    0                                                                  },
	{ &Am29000::SUBC,     0                                                                  },
	{ &Am29000::SUBC,     0                                                                  },
	{ &Am29000::CPBYTE,   IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPBYTE,   IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::SUBRS,    0                                                                  },
	{ &Am29000::SUBRS,    0                                                                  },
	{ &Am29000::SUBRU,    0                                                                  },
	{ &Am29000::SUBRU,    0                                                                  },
	{ &Am29000::SUBR,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SUBR,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::LOADM,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::LOADM,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::SUBRCS,   0                                                                  },
	{ &Am29000::SUBRCS,   0                                                                  },
	{ &Am29000::SUBRCU,   0                                                                  },
	{ &Am29000::SUBRCU,   0                                                                  },
	{ &Am29000::SUBRC,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SUBRC,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::STOREM,   IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::STOREM,   IFLAG_RA_PRESENT                                                   },
	{ &Am29000::CPLT,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPLT,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPLTU,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPLTU,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPLE,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPLE,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPLEU,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPLEU,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPGT,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPGT,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPGTU,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPGTU,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPGE,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPGE,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPGEU,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPGEU,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ASLT,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASLT,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASLTU,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASLTU,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASLE,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASLE,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASLEU,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASLEU,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASGT,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASGT,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASGTU,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASGTU,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASGE,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASGE,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASGEU,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASGEU,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::CPEQ,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPEQ,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::CPNEQ,    IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::CPNEQ,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::MUL,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::MUL,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::MULL,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::MULL,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::DIV0,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::DIV0,     IFLAG_RC_PRESENT                                                   },
	{ &Am29000::DIV,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::DIV,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::DIVL,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::DIVL,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::DIVREM,   IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::DIVREM,   IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ASEQ,     IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASEQ,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ASNEQ,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ASNEQ,    IFLAG_RA_PRESENT                                                   },
	{ &Am29000::MULU,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT             },
	{ &Am29000::MULU,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::INHW,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::INHW,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::EXTRACT,  IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::EXTRACT,  IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::EXHW,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::EXHW,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::EXHWS,    IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::SLL,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SLL,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::SRL,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SRL,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::SRA,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::SRA,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::IRET,     IFLAG_SUPERVISOR_ONLY                                              },
	{ &Am29000::HALT,     IFLAG_SUPERVISOR_ONLY                                              },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::IRETINV,  IFLAG_SUPERVISOR_ONLY                                              },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::AND,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::AND,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::OR,       IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::OR,       IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::XOR,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::XOR,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::XNOR,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::XNOR,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::NOR,      IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::NOR,      IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::NAND,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::NAND,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::ANDN,     IFLAG_RC_PRESENT | IFLAG_RB_PRESENT | IFLAG_RA_PRESENT             },
	{ &Am29000::ANDN,     IFLAG_RC_PRESENT | IFLAG_RA_PRESENT                                },
	{ &Am29000::SETIP,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT | IFLAG_RC_PRESENT             },
	{ &Am29000::INV,      IFLAG_SUPERVISOR_ONLY                                              },
	{ &Am29000::JMP,      0                                                                  },
	{ &Am29000::JMP,      0                                                                  },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPF,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::JMPF,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::CALL,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::CALL,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPT,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::JMPT,     IFLAG_RA_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPFDEC,  IFLAG_RA_PRESENT                                                   },
	{ &Am29000::JMPFDEC,  IFLAG_RA_PRESENT                                                   },
	{ &Am29000::MFTLB,    IFLAG_SUPERVISOR_ONLY | IFLAG_RC_PRESENT | IFLAG_RA_PRESENT        },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::MTTLB,    IFLAG_SUPERVISOR_ONLY | IFLAG_RA_PRESENT | IFLAG_RB_PRESENT        },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPI,     IFLAG_RB_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPFI,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::MFSR,     IFLAG_RC_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::CALLI,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::JMPTI,    IFLAG_RA_PRESENT | IFLAG_RB_PRESENT                                },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::MTSR,     IFLAG_RB_PRESENT                                                   },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::EMULATE,  0                                                                  },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::MULTM,    0                                                                  },
	{ &Am29000::MULTMU,   0                                                                  },
	{ &Am29000::MULTIPLY, 0                                                                  },
	{ &Am29000::DIVIDE,   0                                                                  },
	{ &Am29000::MULTIPLU, 0                                                                  },
	{ &Am29000::DIVIDU,   0                                                                  },
	{ &Am29000::CONVERT,  0                                                                  },
	{ &Am29000::SQRT,     0                                                                  },
	{ &Am29000::CLASS,    0                                                                  },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	// MAME's source table stops at opcode $E9 and relies on zero-filled
	// trailing entries.  A null member-function pointer is not an instruction:
	// the Am29000 architecture routes every unassigned encoding through the
	// illegal-opcode vector.  Fill $EA-$FF explicitly so corrupt/unfilled cache
	// lines cannot jump through address zero in the host process.
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	{ &Am29000::ILLEGAL,  IFLAG_ILLEGAL                                                      },
	// FEQ
	// DEQ
	// TODO! etc
}};
