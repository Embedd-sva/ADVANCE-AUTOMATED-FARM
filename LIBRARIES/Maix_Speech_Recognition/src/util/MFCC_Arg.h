#ifndef	MFCC_Arg
#define	MFCC_Arg
#include "g_def.h"

//汉明窗 窗长160
const u16 hamm[] = {

800, 804, 814, 832, 857, 889, 929, 975, 1028, 1088, 1155, 1228, 1308, 1394, 1486, 1585, 1689, 1800, 1915, 2037,
2163, 2295, 2432, 2573, 2718, 2868, 3022, 3179, 3340, 3504, 3671, 3841, 4013, 4187, 4364, 4542, 4721, 4901, 5082,
5264, 5445, 5627, 5808, 5989, 6169, 6348, 6525, 6700, 6873, 7044, 7213, 7378, 7541, 7700, 7856, 8007, 8155, 8298,
8437, 8571, 8701, 8825, 8943, 9056, 9164, 9265, 9361, 9450, 9533, 9610, 9680, 9743, 9799, 9849, 9892, 9927, 9956,
9978, 9992, 9999, 9999, 9992, 9978, 9956, 9927, 9892, 9849, 9799, 9743, 9680, 9610, 9533, 9450, 9361, 9265, 9164,
9056, 8943, 8825, 8701, 8571, 8437, 8298, 8155, 8007, 7856, 7700, 7541, 7378, 7213, 7044, 6873, 6700, 6525, 6348,
6169, 5989, 5808, 5627, 5445, 5264, 5082, 4901, 4721, 4542, 4364, 4187, 4013, 3841, 3671, 3504, 3340, 3179, 3022,
2868, 2718, 2573, 2432, 2295, 2163, 2037, 1915, 1800, 1689, 1585, 1486, 1394, 1308, 1228, 1155, 1088, 1028, 975,
929, 889, 857, 832, 814, 804, 800
};

#if 0  //1024 point fft
//三角滤波器中心频率点
const u16 tri_cen[] = {

11, 22, 33, 44, 55, 66, 77, 88, 99, 110, 121, 134, 152, 171, 191, 214, 237, 263, 291, 321, 354, 389, 427, 468
};

//奇数三角滤波器首尾相连的折线
const u16 tri_odd[] = {

0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545,
455, 364, 273, 182, 91, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455,
364, 273, 182, 91, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364,
273, 182, 91, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273,
182, 91, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182,
91, 0, 77, 154, 231, 308, 385, 462, 538, 615, 692, 769, 846, 923, 1000, 944, 889, 833, 778, 722, 667, 611, 556,
500, 444, 389, 333, 278, 222, 167, 111, 56, 0, 53, 105, 158, 211, 263, 316, 368, 421, 474, 526, 579, 632, 684,
737, 789, 842, 895, 947, 1000, 950, 900, 850, 800, 750, 700, 650, 600, 550, 500, 450, 400, 350, 300, 250, 200,
150, 100, 50, 0, 43, 87, 130, 174, 217, 261, 304, 348, 391, 435, 478, 522, 565, 609, 652, 696, 739, 783, 826,
870, 913, 957, 1000, 957, 913, 870, 826, 783, 739, 696, 652, 609, 565, 522, 478, 435, 391, 348, 304, 261, 217,
174, 130, 87, 43, 0, 38, 77, 115, 154, 192, 231, 269, 308, 346, 385, 423, 462, 500, 538, 577, 615, 654, 692,
731, 769, 808, 846, 885, 923, 962, 1000, 964, 929, 893, 857, 821, 786, 750, 714, 679, 643, 607, 571, 536, 500,
464, 429, 393, 357, 321, 286, 250, 214, 179, 143, 107, 71, 36, 0, 33, 67, 100, 133, 167, 200, 233, 267, 300,
333, 367, 400, 433, 467, 500, 533, 567, 600, 633, 667, 700, 733, 767, 800, 833, 867, 900, 933, 967, 1000, 970,
939, 909, 879, 848, 818, 788, 758, 727, 697, 667, 636, 606, 576, 545, 515, 485, 455, 424, 394, 364, 333, 303,
273, 242, 212, 182, 152, 121, 91, 61, 30, 0, 29, 57, 86, 114, 143, 171, 200, 229, 257, 286, 314, 343, 371, 400,
429, 457, 486, 514, 543, 571, 600, 629, 657, 686, 714, 743, 771, 800, 829, 857, 886, 914, 943, 971, 1000, 974,
947, 921, 895, 868, 842, 816, 789, 763, 737, 711, 684, 658, 632, 605, 579, 553, 526, 500, 474, 447, 421, 395,
368, 342, 316, 289, 263, 237, 211, 184, 158, 132, 105, 79, 53, 26, 0, 24, 49, 73, 98, 122, 146, 171, 195, 220,
244, 268, 293, 317, 341, 366, 390, 415, 439, 463, 488, 512, 537, 561, 585, 610, 634, 659, 683, 707, 732, 756,
780, 805, 829, 854, 878, 902, 927, 951, 976, 1000, 977, 955, 932, 909, 886, 864, 841, 818, 795, 773, 750, 727,
705, 682, 659, 636, 614, 591, 568, 545, 523, 500, 477, 455, 432, 409, 386, 364, 341, 318, 295, 273, 250, 227,
205, 182, 159, 136, 114, 91, 68, 45, 23, 0
};

//偶数三角滤波器首尾相连的折线
const u16 tri_even[] = {

91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 91,
182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 91, 182,
273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 91, 182, 273,
364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 91, 182, 273, 364,
455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 91, 182, 273, 364, 455,
545, 636, 727, 818, 909, 1000, 923, 846, 769, 692, 615, 538, 462, 385, 308, 231, 154, 77, 0, 56, 111, 167, 222,
278, 333, 389, 444, 500, 556, 611, 667, 722, 778, 833, 889, 944, 1000, 947, 895, 842, 789, 737, 684, 632, 579,
526, 474, 421, 368, 316, 263, 211, 158, 105, 53, 0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600,
650, 700, 750, 800, 850, 900, 950, 1000, 957, 913, 870, 826, 783, 739, 696, 652, 609, 565, 522, 478, 435, 391,
348, 304, 261, 217, 174, 130, 87, 43, 0, 43, 87, 130, 174, 217, 261, 304, 348, 391, 435, 478, 522, 565, 609, 652,
696, 739, 783, 826, 870, 913, 957, 1000, 962, 923, 885, 846, 808, 769, 731, 692, 654, 615, 577, 538, 500, 462,
423, 385, 346, 308, 269, 231, 192, 154, 115, 77, 38, 0, 36, 71, 107, 143, 179, 214, 250, 286, 321, 357, 393, 429,
464, 500, 536, 571, 607, 643, 679, 714, 750, 786, 821, 857, 893, 929, 964, 1000, 967, 933, 900, 867, 833, 800, 767,
733, 700, 667, 633, 600, 567, 533, 500, 467, 433, 400, 367, 333, 300, 267, 233, 200, 167, 133, 100, 67, 33, 0, 30,
61, 91, 121, 152, 182, 212, 242, 273, 303, 333, 364, 394, 424, 455, 485, 515, 545, 576, 606, 636, 667, 697, 727,
758, 788, 818, 848, 879, 909, 939, 970, 1000, 971, 943, 914, 886, 857, 829, 800, 771, 743, 714, 686, 657, 629, 600,
571, 543, 514, 486, 457, 429, 400, 371, 343, 314, 286, 257, 229, 200, 171, 143, 114, 86, 57, 29, 0, 26, 53, 79,
105, 132, 158, 184, 211, 237, 263, 289, 316, 342, 368, 395, 421, 447, 474, 500, 526, 553, 579, 605, 632, 658, 684,
711, 737, 763, 789, 816, 842, 868, 895, 921, 947, 974, 1000, 976, 951, 927, 902, 878, 854, 829, 805, 780, 756, 732,
707, 683, 659, 634, 610, 585, 561, 537, 512, 488, 463, 439, 415, 390, 366, 341, 317, 293, 268, 244, 220, 195, 171,
146, 122, 98, 73, 49, 24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#endif

#if 1  //512 point fft
//三角滤波器中心频率点
const u16 tri_cen[] = {
	4, 7, 12, 16, 21, 26, 32, 38, 44, 51, 59, 67, 76, 85, 96, 107, 119, 132, 146, 161, 177, 195, 213, 234
};

//奇数三角滤波器首尾相连的折线
const u16 tri_odd[] = {

0, 0, 0, 0, 333, 667, 1000, 800, 600, 400, 200, 0, 250, 500, 750, 1000, 800, 600, 400, 200, 0, 200, 400, 600, 800, 1000, 833, 667, 500, 333,
167, 0, 167, 333, 500, 667, 833, 1000, 833, 667, 500, 333, 167, 0, 143, 286, 429, 571, 714, 857, 1000, 875, 750, 625, 500, 375, 250, 125, 0,
125, 250, 375, 500, 625, 750, 875, 1000, 889, 778, 667, 556, 444, 333, 222, 111, 0, 111, 222, 333, 444, 556, 667, 778, 889, 1000, 909, 818,
727, 636, 545, 455, 364, 273, 182, 91, 0, 91, 182, 273, 364, 455, 545, 636, 727, 818, 909, 1000, 917, 833, 750, 667, 583, 500, 417, 333, 250,
167, 83, 0, 77, 154, 231, 308, 385, 462, 538, 615, 692, 769, 846, 923, 1000, 929, 857, 786, 714, 643, 571, 500, 429, 357, 286, 214, 143, 71, 0,
67, 133, 200, 267, 333, 400, 467, 533, 600, 667, 733, 800, 867, 933, 1000, 938, 875, 813, 750, 688, 625, 563, 500, 438, 375, 313, 250, 188,
125, 63, 0, 56, 111, 167, 222, 278, 333, 389, 444, 500, 556, 611, 667, 722, 778, 833, 889, 944, 1000, 944, 889, 833, 778, 722, 667, 611, 556,
500, 444, 389, 333, 278, 222, 167, 111, 56, 0, 48, 95, 143, 190, 238, 286, 333, 381, 429, 476, 524, 571, 619, 667, 714, 762, 810, 857, 905, 952,
1000, 955, 909, 864, 818, 773, 727, 682, 636, 591, 545, 500, 455, 409, 364, 318, 273, 227, 182, 136, 91, 45, 0
};

//偶数三角滤波器首尾相连的折线
const u16 tri_even[] = {

250, 500, 750, 1000, 667, 334, 0, 200, 400, 600, 800, 1000, 750, 500, 250, 0, 200, 400, 600, 800, 1000, 800, 600, 400, 200, 0, 167, 333, 500,
667, 833, 1000, 833, 667, 500, 333, 167, 0, 167, 333, 500, 667, 833, 1000, 857, 714, 571, 429, 286, 143, 0, 125, 250, 375, 500, 625, 750, 875,
1000, 875, 750, 625, 500, 375, 250, 125, 0, 111, 222, 333, 444, 556, 667, 778, 889, 1000, 889, 778, 667, 556, 444, 333, 222, 111, 0, 91, 182,
273, 364, 455, 545, 636, 727, 818, 909, 1000, 909, 818, 727, 636, 545, 455, 364, 273, 182, 91, 0, 83, 167, 250, 333, 417, 500, 583, 667, 750,
833, 917, 1000, 923, 846, 769, 692, 615, 538, 462, 385, 308, 231, 154, 77, 0, 71, 143, 214, 286, 357, 429, 500, 571, 643, 714, 786, 857, 929,
1000, 933, 867, 800, 733, 667, 600, 533, 467, 400, 333, 267, 200, 133, 67, 0, 63, 125, 188, 250, 313, 375, 438, 500, 563, 625, 688, 750, 813,
875, 938, 1000, 944, 889, 833, 778, 722, 667, 611, 556, 500, 444, 389, 333, 278, 222, 167, 111, 56, 0, 56, 111, 167, 222, 278, 333, 389, 444,
500, 556, 611, 667, 722, 778, 833, 889, 944, 1000, 952, 905, 857, 810, 762, 714, 667, 619, 571, 524, 476, 429, 381, 333, 286, 238, 190, 143,
95, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#endif

// 反离散余弦变换 余弦表 24*12
const s8 dct_arg[] = {

	100, 98, 95, 90, 83, 75, 66, 56, 44, 32, 20, 7, -7, -20, -32, -44, -56, -66, -75, -83, -90, -95, -98, -100,
	99, 92, 79, 61, 38, 13, -13, -38, -61, -79, -92, -99, -99, -92, -79, -61, -38, -13, 13, 38, 61, 79, 92, 99,
	98, 83, 56, 20, -20, -56, -83, -98, -98, -83, -56, -20, 20, 56, 83, 98, 98, 83, 56, 20, -20, -56, -83, -98,
	97, 71, 26, -26, -71, -97, -97, -71, -26, 26, 71, 97, 97, 71, 26, -26, -71, -97, -97, -71, -26, 26, 71, 97,
	95, 56, -7, -66, -98, -90, -44, 20, 75, 100, 83, 32, -32, -83, -100, -75, -20, 44, 90, 98, 66, 7, -56, -95,
	92, 38, -38, -92, -92, -38, 38, 92, 92, 38, -38, -92, -92, -38, 38, 92, 92, 38, -38, -92, -92, -38, 38, 92,
	90, 20, -66, -100, -56, 32, 95, 83, 7, -75, -98, -44, 44, 98, 75, -7, -83, -95, -32, 56, 100, 66, -20, -90,
	87, 0, -87, -87, 0, 87, 87, 0, -87, -87, 0, 87, 87, 0, -87, -87, 0, 87, 87, 0, -87, -87, 0, 87,
	83, -20, -98, -56, 56, 98, 20, -83, -83, 20, 98, 56, -56, -98, -20, 83, 83, -20, -98, -56, 56, 98, 20, -83,
	79, -38, -99, -13, 92, 61, -61, -92, 13, 99, 38, -79, -79, 38, 99, 13, -92, -61, 61, 92, -13, -99, -38, 79,
	75, -56, -90, 32, 98, -7, -100, -20, 95, 44, -83, -66, 66, 83, -44, -95, 20, 100, 7, -98, -32, 90, 56, -75,
	71, -71, -71, 71, 71, -71, -71, 71, 71, -71, -71, 71, 71, -71, -71, 71, 71, -71, -71, 71, 71, -71, -71, 71,
};

#endif
