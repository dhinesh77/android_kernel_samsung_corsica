/*****************************************************************
 *
 * Code to be inserted to Rhea code
 * Version:  V0.1
 * Derived from Cori code version V0.7 (25. Apr. 2012)
 *
 * Author: Michael Mayr, Dialog GmbH, May 2nd,. 2012
 *
 *================================================================
 *
 * Input format:
 *      (adc, temp, chg): Voltage, Temperature, Charge indicator
 *      Temperature unit: Kelvin
 *      Voltage unit: ADC codes with adc = (vbat-2500)/2000*2^12 w/ vbat[mV]
 *      Charge indicator:  chg == 0 : discharge
 *                         chg >  0 : charge
 *
 *      Battery current fixed to I_FIXED [mA] defined by battery_data.h
 *
 * Changes (compared to Cori V0.7)
 *      + Removed floating point compile option to enhance readability
 *      + Adapted subsoutine names to Rhea
 *      + Removed DEBUG compile option
 *
 *=================================================================
 *
 * Date: 06.06.2012
 * Version:  V0.2
 *
 * + added charge indicator to the adc_to_soc_with_temp_compensat()
 *   interface
 * + added automatic accuracy adjustment for the low-pass filters
 * + return value is estimated SOC% * 10
 ******************************************************************/

/*****************************************************************
 *
 * Required for adapting to Application:
 *      1. Filter constant needs to be adapted to the application 
 *         dependent on the sample rate, the ADC accuracy and noise
 *      2. Board resistance needs to be adapted (ADDER)
 *      3. Set I_CHG and I_DISCHG  to reasonable values
 *
 *=================================================================
 *
 * Possible code refinements:
 *      1. Code optimization and streamlining
 *      2. Reduce SOC(V,T; I=0) table size w/o loosing too much accuracy
 *
 ******************************************************************/

#include <linux/d2083/battery_data.h>
#include <linux/d2083/appl_data.h>

// Clipping of battery voltage
#define MINMAX_V(x)     x=((x>V_high)?V_high:(x<V_low)?V_low:x)

// Clipping of SOC%
#define MINMAX_S(x)     x=((x>1000)?1000:(x<0)?0:x)

// Improvement for fixed point accuracy
#define PREC_SHIFT      7       // +2 digits decimal precision
#define SHIFT_L(n, x)   ((int)((int)(x) << (n)))        // shift (int)x left by n bits
#define SHIFT_R(n,x)    ((int)((int)(x) >> (n)))        // shift (int)x right by n bits

// ADC -> voltage[mV] conversion
#define CODE2VOLT(x)    (int)(((int)(((int)(x))*2000)>>12) + 2500)

// Function prototypes
u16 temperature_index   (int t);
u16 voltage_index       (int v);
u16 SOC_index           (int s);
int bilin_SOC_Ri        (int S, int T);
int bilin_V_SOC         (int V, int T);
// e/o Function protos

/* int adc_to_soc_with_temp_compensat() interface:
 *      u16 adc: raw adc code
 *      u16 temp: Temperature in Kelvin
 *      u16 chg: charging indicator (chg != 0 : charging)
 *      return value: (int) (SOC% * 10)
 */
//int adc_to_soc_with_temp_compensat (u16 adc, u16 temp, u16 chg)
int adc_to_soc_with_temp_compensat_ant(u16 adc, u16 temp, u16 chg)
{

    static int Iavg, Tavg, Vavg = -1;
    static int V_high = V_HIGH, V_low = V_LOW;

    int Vraw, Traw;
    int i;
    int s, r, v, t;



	//pr_info("%s. volt_raw = %d, temp = %d, chg = %d\n", __func__, adc, temp, chg);

    Vraw = SHIFT_L (PREC_SHIFT, CODE2VOLT (adc));       // Volt[mV] = 2500 + adc*2000/2^12
    Traw = SHIFT_L (PREC_SHIFT, temp);

	//pr_info("%s. Vraw = %d, Traw = %d\n", __func__, Vraw, Traw);

    // Initialization executed once only during the first call
    if (Vavg < 0) {
		Vavg = SHIFT_L (FILTER_CONST, Vraw);
		Tavg = SHIFT_L (FILTER_CONST, Traw);

		//pr_info("%s. Vavg = %d, Tavg = %d\n", __func__, Vavg, Tavg);

		// Scaling of the voltage lookup vector to improve precision
		for (i = 0; voltage[i] < V_high; i++)
		    voltage[i] = SHIFT_L (PREC_SHIFT, voltage[i]);
		voltage[i] = SHIFT_L (PREC_SHIFT, voltage[i]);  // last element

		// Scaling of the temperature lookup vector to improve precision
		for (i = 0; i < N_TEMPERATURE; i++)
		    temperature[i] = SHIFT_L (PREC_SHIFT, temperature[i]);

		// Scaling of Vhigh, Vlow
		V_high = SHIFT_L (PREC_SHIFT, V_high);
		V_low = SHIFT_L (PREC_SHIFT, V_low);
    }

    // LP filter working on powers of 2 => shifts instead of multiply
    Vavg = Vavg + (int) Vraw - SHIFT_R (FILTER_CONST, Vavg);
    Tavg = Tavg + (int) Traw - SHIFT_R (FILTER_CONST, Tavg);
    Iavg = (chg ? I_CHG : I_DISCHG);    // Not averaged because battery current is fixed

	//pr_info("%s. Vavg = %d, Tavg = %d, Iavg = %d\n", __func__, Vavg, Tavg, Iavg);

    v = SHIFT_R (FILTER_CONST, Vavg);
    MINMAX_V (v);

    t = SHIFT_R (FILTER_CONST, Tavg);

    /*
     * First guess  by lookup in table SOC(V, T; I=0)
     * bilinear interpolation is used if U and/or T falls between the table entries
     */

    s = bilin_V_SOC (v, t) / 100;

	//pr_info("%s. first s = %d, v = %d, t = %d\n", __func__, s, v, t);

    MINMAX_S (s);

    // Iteration loop
    for (i = 0; i < 2; i++) {
		r = bilin_SOC_Ri (s, t);

		r += ADDER;             // ADDER for compensating the board resistance

		// Compensated voltage (Ohms law)
		if (Iavg > 0)
		    v = SHIFT_R (FILTER_CONST, Vavg) - SHIFT_L (PREC_SHIFT, r * Iavg / 1000);
		else
		    v = SHIFT_R (FILTER_CONST, Vavg) + SHIFT_L (PREC_SHIFT, r * (-Iavg) / 1000);

		MINMAX_V (v);

		s = bilin_V_SOC (v, t) / 100;
		//pr_info("%s. s(%d) = %d\n",__func__, i, s);

		MINMAX_S (s);
    }

    return ((int) s);           // 10*SOC%
}

/*
 * Find the index of the closest temperature in the temperature[] array
 */
u16 temperature_index (int t)
{
    u16 i;

    if (t <= temperature[0])
	return ((u16) 1);
    if (t > temperature[N_TEMPERATURE - 1])
	return ((u16) (N_TEMPERATURE - 1));

    for (i = 1; t > temperature[i]; i++);

    return ((u16) i);
}

/*
 * Find the index of the closest voltage in the voltage[] array
 */
u16 voltage_index (int v)
{
    u16 i;

    if (v <= voltage[0])
	return ((u16) 1);
    for (i = 1; v > voltage[i]; i++);

    return ((u16) i);
}

/*
 * Find the index of the closest SOC in the SOC[] array
 */
u16 SOC_index (int s)
{
    int i;

    if (s <= 0)
	return ((u16) 1);
    for (i = 1; s > SOC[i]; i++);

    return ((u16)i);
}


/*
 * Bilinear interpolation for Ri(SOC, T)
 */
int bilin_SOC_Ri (int S, int T)
{
    u16 t_index, s_index;
    int x = S, y = T, x1, x2, y1, y2, z11, z12, z21, z22, ri;
    int r1, r2;

    t_index = temperature_index (T);
    s_index = SOC_index (S);

	//pr_info("%s. t_index(%d), v_index(%d)\n", __func__, t_index, s_index);

    x1 = SOC[s_index - 1];
    x2 = SOC[s_index];
    y1 = temperature[t_index - 1];
    y2 = temperature[t_index];

    z11 = RI_SOCT[s_index - 1][t_index - 1];
    z12 = RI_SOCT[s_index - 1][t_index];
    z21 = RI_SOCT[s_index][t_index - 1];
    z22 = RI_SOCT[s_index][t_index];

    // bilinear interpolation formula replaced by tripple linear interpolation to avoid integer overflow
    r1 = SHIFT_L (PREC_SHIFT, (z21 - z11)) / (x2 - x1) * (x - x1) + SHIFT_L (PREC_SHIFT, z11);
    r2 = SHIFT_L (PREC_SHIFT, (z22 - z12)) / (x2 - x1) * (x - x1) + SHIFT_L (PREC_SHIFT, z12);
    ri = (r2 - r1) / (y2 - y1) * (y - y1) + r1;

    ri = SHIFT_R (PREC_SHIFT, ri);

    return ((int)ri);           // 1000*Ohms
}

/*
 * Bilinear interpolation for SOC(V, T; I=0)
 */
int bilin_V_SOC (int V, int T)
{

    int x = V, y = T, x1, x2, y1, y2, z11, z12, z21, z22, s;
    int soc1, soc2;
    u16 t_index, v_index;

    t_index = temperature_index (T);
    v_index = voltage_index (V);

	//pr_info("%s. t_index(%d), v_index(%d)\n", __func__, t_index, v_index);

    x1 = voltage[v_index - 1];
    x2 = voltage[v_index];
    y1 = temperature[t_index - 1];
    y2 = temperature[t_index];

    z11 = SOC_VT[v_index - 1][t_index - 1];
    z12 = SOC_VT[v_index - 1][t_index];
    z21 = SOC_VT[v_index][t_index - 1];
    z22 = SOC_VT[v_index][t_index];

    // bilinear interpolation formula replaced by tripple linear interpolation to avoid integer overflow
    soc1 = SHIFT_L (PREC_SHIFT, (z21 - z11)) / (x2 - x1) * (x - x1) + SHIFT_L (PREC_SHIFT, z11);
    soc2 = SHIFT_L (PREC_SHIFT, (z22 - z12)) / (x2 - x1) * (x - x1) + SHIFT_L (PREC_SHIFT, z12);
    s = (soc2 - soc1) / (y2 - y1) * (y - y1) + soc1;

    s = SHIFT_R (PREC_SHIFT, s);

    return ((int)s);                    // 1000*SOC%
}
