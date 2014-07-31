/*
 * Application specific entries i.E. battery contact resistance
 *
 * Apr. 22, 2012: Modified FILTER_CONST for fixed point implementation
 *
 * For Rhea:
 * May 2nd, 2012: Removed floating point data including related compiler option
 *
 * June 6th, 2012:
 *   Setting optimized for use with Vostro platform!
 *   I_DISCHG and I_CHG moved to here
 */

#define APPL_DATA

#ifndef ADDER
#define ADDER	(110)		// Kikusui test board contact and cabeling series resistance [mOhms]
#endif

/*
 * LP filtering of the raw current and voltage values
 * should be at least 1/16 or bigger for smoothing the data
 */
#ifndef FILTER_CONST
#define FILTER_CONST   (5)	// 2**(-5) = 0.03125
#endif

/*
 * Fixed currents for charge and discharge
 */
#ifndef I_CHG
#define I_CHG	((short)250)   // fixed 250mA charge current
#endif

#ifndef I_DISCHG
#define I_DISCHG	((short)-100)   // fixed -100mA discharge current
#endif

