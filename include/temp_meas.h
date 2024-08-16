/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef TEMP_MEAS_H_INCLUDED
#define TEMP_MEAS_H_INCLUDED

class TempMeas
{
public:
   enum Sensors
   {
      TEMP_JCURVE = 0,
      TEMP_KTY81 = 1,
      TEMP_KTY83 = 2,
      TEMP_NTC50K = 3,
      TEMP_TESLA_10K = 4,
      TEMP_PT1000 = 5,
      TEMP_LAST
   };

   static float Lookup(int digit, Sensors sensorId);
};


#ifdef __TEMP_LU_TABLES
#define JCURVE \
57	,\
76	,\
100	,\
132	,\
171	,\
220	,\
280	,\
353	,\
440	,\
544	,\
665	,\
805	,\
963	,\
1141	,\
1338	,\
1551	,\
1779	,\
2019	,\
2268	,\
2523	,\
2779	,\
3032	,\
3279	,\
3519	,\
3748	,\
3964	,\
4167

#define KTY81_110 \
2062	,\
1994	,\
1925	,\
1857	,\
1791	,\
1725	,\
1661	,\
1598	,\
1536	,\
1478	,\
1420	,\
1365	,\
1312	,\
1261	,\
1212	,\
1165	,\
1120	,\
1078	,\
1040	,\
1007	,\
980	



#define KTY83_110 \
2035	,\
1968	,\
1901	,\
1835	,\
1769	,\
1705	,\
1643	,\
1582	,\
1522	,\
1465	,\
1409	,\
1356	,\
1304	,\
1255	,\
1208	,\
1162	,\
1119	,\
1077	,\
1037	,\
999	,\
962	,\
927	,\
894	

#define NPT18_50K \
3978	,\
3963	,\
3947	,\
3929	,\
3909	,\
3887	,\
3864	,\
3838	,\
3810	,\
3779	,\
3746	,\
3710	,\
3672	,\
3631	,\
3588	,\
3541	,\
3492	,\
3440	,\
3385	,\
3327	,\
3266	,\
3203	,\
3137	,\
3069	,\
2999	,\
2927	,\
2853	,\
2777	,\
2700	,\
2622	,\
2543	,\
2463	,\
2383	,\
2303	,\
2223	,\
2144	,\
2065	,\
1987	,\
1910	,\
1835	,\
1761	,\
1688	,\
1618	,\
1549	,\
1482	,\
1417	,\
1354	,\
1293	,\
1235	,\
1178	,\
1124	,\
1072	,\
1022	,\
974	,\
928	,\
884	,\
843	,\
803	,\
764	,\
728	,\
694	,\
661	,\
629	,\
600	,\
571	,\
544	,\
519	,\
494	,\
471	,\
449	,\
428	,\
409	,\
390	,\
372	,\
355	,\
339	,\
324	,\
309	,\
295	,\
282	,\
270	



#define TESLA_10K \
4005	,\
3989	,\
3970	,\
3949	,\
3925	,\
3897	,\
3866	,\
3831	,\
3791	,\
3747	,\
3698	,\
3643	,\
3583	,\
3518	,\
3446	,\
3369	,\
3286	,\
3198	,\
3104	,\
3005	,\
2902	,\
2794	,\
2683	,\
2570	,\
2455	,\
2338	,\
2222	,\
2105	,\
1990	,\
1877	,\
1766	,\
1658	,\
1554	,\
1454	,\
1358	,\
1267	,\
1180	,\
1098	,\
1020	,\
947	,\
879	,\
815	,\
755	,\
700	,\
648	,\
600	,\
556	,\
515	,\
477	,\
441	,\
409	,\
379	,\
351	,\
325	,\
302	,\
280	,\
260	,\
241	,\
224	,\
208	,\
194	,\
180	,\
168	,\
157	,\
146	,\
136	,\
127	,\
119	,\
111	,\
104	,\
97	






#define PT1000 \
2488, \
2560, \
2629, \
2696, \
2760, \
2821, \
2880, \
2937, \
2991, \
3044, \
3095, \
3144, \
3192, \
3238, \
3282, \
3325, \
3367, \
3407, \
3446, \
3484, \
3521

#endif

#endif // TEMP_MEAS_H_INCLUDED
