// Code for MakeIt Venty
// by Nick Serpa, Jonathan Vail, Phil Martel
// License ??
/* Brief description:
 * Sinusoid table for venty
 * SIN_VAL_SIZE is the number of points aroiund the circle
 * The table has an extra point to close off the circle
 * sinVal[0] == sinVal[24]
 * Currrently,  the values are generated by venty-table.xls
 * and copied into this table
 */
#ifndef VENTY_TABLE_H
#define VENTY_TABLE_H

#define SIN_VAL_SIZE 24
float sinVal[SIN_VAL_SIZE +1] = {
1	,
0.982962913144534	,
0.933012701892219	,
0.853553390593274	,
0.75	,
0.62940952255126	,
0.5	,
0.37059047744874	,
0.25	,
0.146446609406726	,
0.066987298107781	,
0.017037086855466	,
0	,
0.017037086855466	,
0.066987298107781	,
0.146446609406726	,
0.25	,
0.37059047744874	,
0.5	,
0.629409522551261	,
0.75	,
0.853553390593274	,
0.933012701892219	,
0.982962913144534	,
1	
};
#endif
