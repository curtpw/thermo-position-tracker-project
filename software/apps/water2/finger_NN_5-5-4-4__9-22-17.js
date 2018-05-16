function fingerNNetwork(input){
F = {
/*	F[3]=input[0];
	F[5]=input[1];
	F[7]=input[2];
	F[9]=input[3];
	F[11]=input[4];
 */


3: 0.8227722772277227,
5: 0.8554455445544555,
7: 0.8168316831683168,
9: 0.8297029702970297,
11: 0.8425742574257425,
23: 0.7516646885298827,
0: 1.105743123941634,
1: 1.1075104362664407,
2: 1.0096270923780082,
4: -0.014328760895130474,
6: -0.007558445957053908,
8: -0.026156648318406778,
10: -0.02203919932208444,
12: 0.06603167983521403,
13: 0.999998334530251,
14: 0.06539827283793272,
15: 0.9999983384338412,
16: 0.015450623638136303,
17: 0.9999983964257433,
18: -0.016726550816954882,
19: 0.9999983311496663,
20: 0.0802692318657802,
21: 0.9999983960553631,
22: -0.044238636095071844,
24: 0,
190: 13.293537530570656,
194: 0.20492523949360816,
195: 0.3529583225640364,
196: 0.2602483352794215,
197: 0.2510409625303785,
198: 0.2385059229416253,
192: 1,
125: 1,
25: 0.7516646885298827,
26: 0.7516646885298827,
27: 0.7516646885298827,
28: 0.7516646885298827,
29: 0.7516646885298827,
43: 0.7881445608126791,
30: 1.313166209590327,
31: 1.3137773743874255,
32: 1.007701401581027,
33: 0.06066529404870203,
34: -0.040925335743802596,
35: 0.03587766052602296,
36: 0.05971036125136734,
37: 0.0923310896541515,
38: 0.0818734270988138,
39: -0.04848527358494953,
40: 0.0336052146282477,
41: 0.113733465528541,
42: -0.046199258373306344,
44: 0,
200: 13.295306722358864,
204: 0.3131010106254193,
205: 0.2994464477121029,
206: 0.23587908464756555,
207: 0.24085765921961874,
208: 0.29145508920059676,
202: 1,
141: 1,
45: 0.7881445608126791,
46: 0.7881445608126791,
47: 0.7881445608126791,
48: 0.7881445608126791,
49: 0.7881445608126791,
63: 0.7768973315553632,
50: 1.2495918122394767,
51: 1.247676145093907,
52: 1.0061371658814515,
53: 0.0844232489673779,
54: 0.10265897827573611,
55: -0.037835251607999866,
56: -0.051770385449622654,
57: 0.030343211612768067,
58: -0.042722859932933066,
59: 0.060609660623694266,
60: 0.04577553467744228,
61: 0.12188832595024893,
62: -0.05299969976937183,
64: 0,
210: 13.33198278071767,
214: 0.20159004162492444,
215: 0.28402931113246477,
216: 0.23974452224632298,
217: 0.2952813525169763,
218: 0.22298328240428442,
212: 1,
157: 1,
65: 0.7768973315553632,
66: 0.7768973315553632,
67: 0.7768973315553632,
68: 0.7768973315553632,
69: 0.7768973315553632,
83: 0.8153021820933861,
70: 1.4859694215009858,
71: 1.4848377472175271,
72: 1.0064351468059103,
73: 0.07313811993384838,
74: -0.0021370385148660465,
75: 0.10114971912224967,
76: 0.1037603484462141,
77: 0.06518847764970549,
78: -0.030351131001136927,
79: 0.07360390417885528,
80: 0.05297715686706551,
81: 0.09205968622249149,
82: 0.008126183178982914,
84: 0,
220: 13.291566830768147,
224: 0.27836030711708964,
225: 0.3320204958624524,
226: 0.21996833070279367,
227: 0.29457352551473986,
228: 0.18646040570676173,
222: 1,
173: 1,
85: 0.8153021820933861,
86: 0.8153021820933861,
87: 0.8153021820933861,
88: 0.8153021820933861,
89: 0.8153021820933861,
103: 0.8066481623446623,
90: 1.4298309084098073,
91: 1.4283760686747398,
92: 1.009289529976635,
93: 0.13194491840649017,
94: -0.0444026688440959,
95: 0.02653422677973344,
96: -0.021679016130305102,
97: 0.011771115223822822,
98: 0.037145136993008844,
99: 0.1123313208439826,
100: 0.030159415871050456,
101: 0.0944268613926508,
102: 0.06084228127953593,
104: 0,
230: 13.33136441427025,
234: 0.2891682402062779,
235: 0.3733858241282532,
236: 0.27810795047999154,
237: 0.2880872066006155,
238: 0.238786449217146,
232: 1,
189: 1,
105: 0.8066481623446623,
106: 0.8066481623446623,
107: 0.8066481623446623,
108: 0.8066481623446623,
109: 0.8066481623446623,
123: 0.9325240208131698,
110: 2.632796907010304,
111: 2.6261232401262777,
112: 1.032733789499175,
113: 0.18938528537422322,
114: 0.21273507365446803,
115: 0.24138434908422093,
116: 0.26829476767800425,
117: 0.1249621445317956,
118: 0.23877420923480916,
119: 0.13188195019526916,
120: 0.11214401080436152,
121: 0.11890246214146967,
122: 0.1288154757426953,
124: 0,
139: 0.9324796080936423,
126: 2.629998457205187,
127: 2.625417628711591,
128: 1.0289248455875601,
129: 0.1302939587740918,
130: 0.19285454349924086,
131: 0.23122486056748776,
132: 0.15546459947884733,
133: 0.12226692547253734,
134: 0.19602190646868387,
135: 0.20455446362466492,
136: 0.21396200346391736,
137: 0.13195744363079948,
138: 0.1569365041569662,
140: 0,
155: 0.9296889391153457,
142: 2.5832818636472585,
143: 2.581920931357823,
144: 1.0345406198684894,
145: 0.06528837681879104,
146: 0.15434096520997428,
147: 0.19505518952737785,
148: 0.13613455186383655,
149: 0.19443408569241818,
150: 0.12642428464368505,
151: 0.26340548146882176,
152: 0.10821791059442226,
153: 0.17027455738077993,
154: 0.2572048818403433,
156: 0,
171: 0.9246178081931278,
158: 2.5121422738892605,
159: 2.5068094076125575,
160: 1.0404270575409962,
161: 0.22716553684078913,
162: 0.14506027961998327,
163: 0.08352390197806592,
164: 0.15790655517028224,
165: 0.11610164208343017,
166: 0.12880855605615582,
167: 0.23679421184575983,
168: 0.1565466923296651,
169: 0.0984066225814871,
170: 0.23776301467734828,
172: 0,
187: 0.9251304305023637,
174: 2.519325605250842,
175: 2.5141872072926397,
176: 1.0404667833599035,
177: 0.23970335317668306,
178: 0.09569233309800164,
179: 0.10717234791021041,
180: 0.13399268930289898,
181: 0.09579555136433365,
182: 0.11995126571747627,
183: 0.274155227941506,
184: 0.13699980993869318,
185: 0.2901755409122461,
186: 0.0939251786245388,
188: 0,
191: 13.305401677294736,
193: 0.08776116296154504,
199: 0,
690: 3.430377120301015,
255: 0.30286142858952314,
709: -3.3326227958957206,
256: 0.30286142858952314,
728: -0.594261665115369,
257: 0.30286142858952314,
747: -0.6724330663353755,
258: 0.30286142858952314,
248: 0.5698128820714863,
267: 0.5717501336337534,
286: 1.3933011972395057,
305: 1.1180586825129386,
324: 1.3785863503888205,
544: 0.07654459788899194,
363: 0.7874257239500039,
559: 0.08769780515855233,
392: 0.7922622876169138,
574: 0.1854478974062421,
421: 0.8093575984758012,
589: 0.18654930932791017,
450: 0.7704098799056832,
343: -0.016544787391199255,
376: 0.08005608910870317,
405: -0.024650760432927663,
434: 0.0996802345107211,
463: 0.019967678287869446,
483: 0.014818302460681157,
503: 0.06774320275971685,
523: 0.08592599288682581,
603: -0.22627667006438323,
626: -0.10700737757761103,
649: 0.03232520320913137,
672: -0.0367216533348313,
201: 13.307748269714924,
203: 0.0023149186310929977,
209: 0,
691: 3.3006959453889912,
274: 0.2715892486883272,
710: -3.2484208443191545,
275: 0.2715892486883272,
729: -0.8827280653148541,
276: 0.2715892486883272,
748: -0.8803635727597389,
277: 0.2715892486883272,
249: 0.5408596121707292,
268: 0.5490665928809821,
287: 1.288653081402756,
306: 1.1656963790484731,
325: 1.3694482365203176,
545: 0.06912308773945647,
364: 0.7874257239500039,
560: 0.09735314345265722,
393: 0.7922622876169138,
575: 0.1644359025896377,
422: 0.8093575984758012,
590: 0.23385049645682177,
451: 0.7704098799056832,
344: 0.03733477527037692,
377: 0.05929791629177043,
406: 0.07552433413699057,
435: -0.008406524776639264,
464: 0.04225375493642664,
484: 0.006767843286179479,
504: 0.05547828380474925,
524: 0.1896165428317375,
604: -0.2303368571805519,
627: -0.19040865825429637,
650: 0.02044923565092277,
673: -0.10630049341064812,
211: 13.343273906136538,
213: 0.1426132360893647,
219: 0,
692: 5.319474315293542,
293: 0.3238683240087867,
711: -5.351236299265023,
294: 0.3238683240087867,
730: -0.9895632561546218,
295: 0.3238683240087867,
749: -0.9526088192073672,
296: 0.3238683240087867,
250: 0.6321552904670918,
269: 0.48615882976261376,
288: 1.4162912354761032,
307: 1.020887329067313,
326: 1.4806749924213285,
546: 0.11039499271657056,
365: 0.7874257239500039,
561: 0.1120081243791505,
394: 0.7922622876169138,
576: 0.16350135174598734,
423: 0.8093575984758012,
591: 0.12423919278985156,
452: 0.7704098799056832,
345: 0.028272137218994374,
378: 0.01728854292918081,
407: 0.11141153920984427,
436: 0.03758028079989343,
465: 0.16502412784806658,
485: 0.16016758039882512,
505: 0.16101404354695179,
525: 0.02156883266534376,
605: -0.14274552637786905,
628: -0.16029232677016045,
651: -0.10813267042105999,
674: -0.0469319945073969,
221: 13.303373922625337,
223: 0.12161331943127801,
229: 0,
693: 4.584837429546359,
312: 0.35824107342950934,
712: -4.658809735453228,
313: 0.35824107342950934,
731: -0.9344469587742736,
314: 0.35824107342950934,
750: -0.992343494564046,
315: 0.35824107342950934,
251: 0.5322230635677215,
270: 0.4685914095806357,
289: 1.3356144256108473,
308: 0.9958242216234102,
327: 1.3709775012008223,
547: 0.2076105625580972,
366: 0.7874257239500039,
562: 0.11189875902727602,
395: 0.7922622876169138,
577: 0.14891039864256397,
424: 0.8093575984758012,
592: 0.2263227637988057,
453: 0.7704098799056832,
346: 0.03799104126322846,
379: -0.0003387291955966613,
408: 0.07131664442773408,
437: 0.07862901418507198,
466: 0.14867681774810784,
486: 0.011268860784886512,
506: 0.11881197426137292,
526: 0.032500693668224244,
606: -0.13530499858861464,
629: -0.12148417948558754,
652: 0.031607189613370786,
675: -0.08129194672919177,
231: 13.343042960868782,
233: 0.021872722840355352,
239: 0,
694: 5.395316797287665,
331: 0.3243245753457321,
713: -5.375341579010414,
332: 0.3243245753457321,
732: -0.7379850664041204,
333: 0.3243245753457321,
751: -0.8400186611558129,
334: 0.3243245753457321,
252: 0.6974705957242514,
271: 0.4596406622575157,
290: 1.4168259724617216,
309: 1.0570277589512485,
328: 1.438571113603172,
548: 0.09106532716082474,
367: 0.7874257239500039,
563: 0.14930597924050842,
396: 0.7922622876169138,
578: 0.04528959463878825,
425: 0.8093575984758012,
593: 0.05833302697904464,
454: 0.7704098799056832,
347: 0.04292125703498034,
380: -0.07326867898237838,
409: -0.0686939198292632,
438: -0.016899233906337283,
467: 0.08965737241465993,
487: 0.1336016512961747,
507: 0.08580663999854478,
527: 0.10549798964549865,
607: -0.19447249066905595,
630: -0.12243623638753995,
653: -0.13391336095326528,
676: -0.1903963199962133,
253: 0.30286142858952314,
240: -1.1278181892262937,
241: -0.8337088326922815,
242: 9.915537661663132,
243: 2.16291348605001,
244: -23.7944231055248,
245: 6.155963041770239,
246: -1.104825251329839,
247: 0.8591167183611005,
254: 0,
687: -5.785843435236739,
706: 5.788926110055662,
725: -10.299890227843223,
744: -10.279379188282006,
272: 0.2715892486883272,
259: -1.2634057071776579,
260: -0.9865742981132948,
261: 9.819247077755724,
262: 1.9507647840826723,
263: -22.501219247398407,
264: 5.709245813923481,
265: -1.0744128636773658,
266: 0.6102876560200624,
273: 0,
291: 0.3238683240087867,
278: -1.205100056281652,
279: -0.7360508182375158,
280: 13.23540240781642,
281: 4.002888902396751,
282: -38.56101223585313,
283: 10.373402101445802,
284: -1.2950005659280692,
285: 1.7127197258765703,
292: 0,
310: 0.35824107342950934,
297: -0.9895010900843795,
298: -0.5830065801428308,
299: 12.483646834844697,
300: 3.4174481017296396,
301: -33.62615166312728,
302: 8.974127701347932,
303: -1.1032653071675718,
304: 1.2910470996965093,
311: 0,
329: 0.3243245753457321,
316: -1.2030770333445007,
317: -0.7339680295860735,
318: 13.094884612208656,
319: 4.198241548044156,
320: -38.86913818174601,
321: 10.41776551778875,
322: -1.2533731625145017,
323: 1.6970651137160344,
330: 0,
356: 0.7874257239500039,
335: 1.3102021492286813,
336: 1.3094775858133514,
337: 1.0055208932152602,
338: -0.008353367686677418,
339: 0.03992947703669542,
340: -0.04778517505742032,
341: 0.10902456657273901,
342: 0.016476218282857398,
348: 0.999887329106545,
349: 0.06995907091349954,
350: 0.9998965910612919,
351: 0.04356532061783846,
352: 0.999934943622828,
353: -0.00847997458807531,
354: 0.9999091268263649,
355: -0.023645077158488688,
357: 0,
535: 9.084404582892232,
539: 0.08878317453222467,
540: 0.1860806005400061,
541: 0.1881526746004058,
542: 0.22885027142004394,
543: 0.20585408703983976,
537: 1,
474: 1,
358: 0.7874257239500039,
359: 0.7874257239500039,
360: 0.7874257239500039,
361: 0.7874257239500039,
362: 0.7874257239500039,
385: 0.7922622876169138,
368: 1.338406310788593,
369: 1.338616222406923,
370: 1.006083766176409,
371: -0.0741089457379793,
372: 0.0522927904581761,
373: 0.054221638819483074,
374: 0.11469599523060629,
375: 0.03198190162533273,
381: 0.08900019119020823,
382: -0.06209497487254652,
383: 0.009057534462529607,
384: 0.06338471013359849,
386: 0,
550: 9.171154924505835,
554: 0.2123422106620405,
555: 0.2680890017912282,
556: 0.18898313385272977,
557: 0.09983361429055054,
558: 0.17388223424242286,
552: 1,
494: 1,
387: 0.7922622876169138,
388: 0.7922622876169138,
389: 0.7922622876169138,
390: 0.7922622876169138,
391: 0.7922622876169138,
414: 0.8093575984758012,
397: 1.4492755797326389,
398: 1.4458414148244751,
399: 1.0030296010324615,
400: 0.0822258660350605,
401: 0.036325596739888305,
402: 0.018733201383997113,
403: 0.07007112068975871,
404: -0.051851469302159774,
410: 0.017088260090636312,
411: 0.10018762096627638,
412: 0.07112408288365119,
413: -0.03896198373137871,
415: 0,
565: 9.634653770096952,
569: 0.15509891189512073,
570: 0.04299984554122314,
571: 0.17239766728478015,
572: 0.09530628872729886,
573: 0.0840638976834448,
567: 1,
514: 1,
416: 0.8093575984758012,
417: 0.8093575984758012,
418: 0.8093575984758012,
419: 0.8093575984758012,
420: 0.8093575984758012,
443: 0.7704098799056832,
426: 1.2131302273619737,
427: 1.2106270522145524,
428: 1.0051094394564148,
429: 0.03976752423921703,
430: 0.04126285721941597,
431: -0.013844724725132675,
432: 0.018906937934054527,
433: -0.0635432601409068,
439: -0.07257280942611116,
440: -0.06499922460280674,
441: 0.04831331846100423,
442: 0.08533212599485628,
444: 0,
580: 9.300171518093482,
584: 0.17981114075653018,
585: 0.06026089646859071,
586: 0.064805820986821,
587: 0.12502073042543413,
588: 0.14519011305575116,
582: 1,
534: 1,
445: 0.7704098799056832,
446: 0.7704098799056832,
447: 0.7704098799056832,
448: 0.7704098799056832,
449: 0.7704098799056832,
472: 0.8859492397409454,
455: 2.0494397138658593,
456: 2.0500160427515217,
457: 1.0233047124121781,
458: 0.024478678670438368,
459: 0.025587590241939306,
460: -0.005061435605403458,
461: 0.10562325351222533,
462: 0.1169405891126472,
468: 0.11263821637654893,
469: -0.0035420570886702998,
470: 0.0856637355247685,
471: 0.14234009831712965,
473: 0,
492: 0.8814828494685762,
475: 2.012241362310166,
476: 2.0065478652464033,
477: 1.0269856920413694,
478: 0.17838458096194737,
479: 0.18281773891578973,
480: 0.10776453545522943,
481: 0.028031431773435853,
482: 0.029968869600727307,
488: 0.10008306351756839,
489: 0.04939907883375236,
490: 0.021014220294849027,
491: 0.04276679646349841,
493: 0,
512: 0.8893134058079787,
495: 2.0837308285078953,
496: 2.083746978815691,
497: 1.0204905960971047,
498: 0.025033862707652392,
499: 0.12940395183695302,
500: 0.08287544741643449,
501: 0.09806620261347881,
502: 0.1785481053349286,
508: 0.11167650537391266,
509: 0.007652979271718293,
510: -0.0037621809679630478,
511: 0.028051613545689007,
513: 0,
532: 0.8848538999813378,
515: 2.0427673272872098,
516: 2.0392207896245798,
517: 1.0241351614906538,
518: 0.17628467942511705,
519: 0.05132532090907031,
520: 0.06406603664075136,
521: 0.15791681072394545,
522: 0.0892995507178048,
528: 0.05372300190846616,
529: 0.042363410216498705,
530: 0.032549248159283534,
531: 0.0038061906368572073,
533: 0,
536: 9.090926758739215,
538: 0.015815848780831778,
549: 0,
695: 0.9584233973584899,
614: 0.10204547524031617,
714: -0.938487106562026,
615: 0.10204547524031617,
733: -0.6429255399521543,
616: 0.10204547524031617,
752: -0.6742071469870838,
617: 0.10204547524031617,
608: -0.20037025033483283,
631: -0.22095658329374732,
654: -0.032232256424300325,
677: -0.036894916825607596,
551: 9.176715737486012,
553: 0.026101242735724613,
564: 0,
696: 1.0623647365418134,
637: 0.10860675857331685,
715: -1.1306070979900236,
638: 0.10860675857331685,
734: -0.7177741260416682,
639: 0.10860675857331685,
753: -0.6048379592457217,
640: 0.10860675857331685,
609: -0.2864703608680945,
632: -0.17810699882765121,
655: 0.031759957675925335,
678: -0.03739822231648722,
566: 9.64019126439133,
568: 0.1309188929775762,
579: 0,
697: 1.5931218117630694,
660: 0.13035200083478182,
716: -1.6452627712762682,
661: 0.13035200083478182,
735: -0.7795436234558345,
662: 0.13035200083478182,
754: -0.8834581362916947,
663: 0.13035200083478182,
610: -0.2702667697554752,
633: -0.15326064620694202,
656: 0.06301337782223368,
679: -0.05878052385233666,
581: 9.305954842559153,
583: 0.06912961768896608,
594: 0,
698: 1.4172197216296334,
683: 0.12211976360685073,
717: -1.4146385226352916,
684: 0.12211976360685073,
736: -0.8374209528056362,
685: 0.12211976360685073,
755: -0.705749637958328,
686: 0.12211976360685073,
611: -0.20496130321708964,
634: -0.21090796743249648,
657: -0.086617549782322,
680: -0.16158063547634938,
612: 0.10204547524031617,
595: -2.2264848069266048,
596: -2.1747008768220617,
597: 3.0755241429615525,
598: 0.03399471489723571,
599: -3.929865995422341,
600: 0.6707665599838437,
601: -0.5083754153989416,
602: -0.18335967752868354,
613: 0,
635: 0.10860675857331685,
618: -2.16879854612384,
619: -2.105052039304292,
620: 3.429733865077842,
621: 0.23270291271734395,
622: -4.992794918660496,
623: 0.8647989769123039,
624: -0.6737588187702918,
625: -0.16800376308942264,
636: 0,
658: 0.13035200083478182,
641: -2.0164107170618966,
642: -1.8978500411434938,
643: 5.0977808178046145,
644: 0.6766147587661829,
645: -9.409072269701923,
646: 1.9248972397024207,
647: -1.0481997051897314,
648: -0.036569066310029145,
659: 0,
681: 0.12211976360685073,
664: -2.070310702347159,
665: -1.9725079471918636,
666: 4.435216729394897,
667: 0.4697693679726529,
668: -7.592567633201606,
669: 1.4889965448179028,
670: -0.9033091852669,
671: -0.017960925905872897,
682: 0,
704: 0.02649079712506273,
688: -3.6041098837797536,
689: 3.9568866038684716,
699: 5.086541224667357,
700: -31.29158565538645,
701: 9.782397564774403,
702: -2.7214443700664943,
703: 1.9447542688437711,
705: 0,
723: 0.9735431353631449,
707: 3.605426476181405,
708: -4.017755904325545,
718: -5.059973066174672,
719: 31.30161092372356,
720: -9.744810874513977,
721: 2.7577868785637207,
722: -1.9812510450377239,
724: 0,
742: 0.000024709868910778283,
726: -10.608283132911746,
727: -1.8851713257571732,
737: -1.7116107413899295,
738: -1.7377892666605153,
739: -1.6828078639537014,
740: -1.612561558209594,
741: -1.72569911269762,
743: 0,
761: 0.000024787516161031006,
745: -10.6051456243121,
746: -2.04676576640198,
756: -1.6193289730334401,
757: -1.6522865853792226,
758: -1.5970918687180784,
759: -1.6441597941910606,
760: -1.6958442457869292,
762: 0
};


	F[3]=input[0];
	F[5]=input[1];
	F[7]=input[2];
	F[9]=input[3];
	F[11]=input[4];


F[0] = F[1];F[1] = F[2];F[1] += F[3] * F[4];F[1] += F[5] * F[6];F[1] += F[7] * F[8];F[1] += F[9] * F[10];F[1] += F[11] * F[12];F[1] += F[13] * F[14];F[1] += F[15] * F[16];F[1] += F[17] * F[18];F[1] += F[19] * F[20];F[1] += F[21] * F[22];F[23] = (1 / (1 + Math.exp(-F[1])));F[24] = F[23] * (1 - F[23]);F[25] = F[23];F[26] = F[23];F[27] = F[23];F[28] = F[23];F[29] = F[23];
F[30] = F[31];F[31] = F[32];F[31] += F[3] * F[33];F[31] += F[5] * F[34];F[31] += F[7] * F[35];F[31] += F[9] * F[36];F[31] += F[11] * F[37];F[31] += F[13] * F[38];F[31] += F[15] * F[39];F[31] += F[17] * F[40];F[31] += F[19] * F[41];F[31] += F[21] * F[42];F[43] = (1 / (1 + Math.exp(-F[31])));F[44] = F[43] * (1 - F[43]);F[45] = F[43];F[46] = F[43];F[47] = F[43];F[48] = F[43];F[49] = F[43];
F[50] = F[51];F[51] = F[52];F[51] += F[3] * F[53];F[51] += F[5] * F[54];F[51] += F[7] * F[55];F[51] += F[9] * F[56];F[51] += F[11] * F[57];F[51] += F[13] * F[58];F[51] += F[15] * F[59];F[51] += F[17] * F[60];F[51] += F[19] * F[61];F[51] += F[21] * F[62];F[63] = (1 / (1 + Math.exp(-F[51])));F[64] = F[63] * (1 - F[63]);F[65] = F[63];F[66] = F[63];F[67] = F[63];F[68] = F[63];F[69] = F[63];
F[70] = F[71];F[71] = F[72];F[71] += F[3] * F[73];F[71] += F[5] * F[74];F[71] += F[7] * F[75];F[71] += F[9] * F[76];F[71] += F[11] * F[77];F[71] += F[13] * F[78];F[71] += F[15] * F[79];F[71] += F[17] * F[80];F[71] += F[19] * F[81];F[71] += F[21] * F[82];F[83] = (1 / (1 + Math.exp(-F[71])));F[84] = F[83] * (1 - F[83]);F[85] = F[83];F[86] = F[83];F[87] = F[83];F[88] = F[83];F[89] = F[83];
F[90] = F[91];F[91] = F[92];F[91] += F[3] * F[93];F[91] += F[5] * F[94];F[91] += F[7] * F[95];F[91] += F[9] * F[96];F[91] += F[11] * F[97];F[91] += F[13] * F[98];F[91] += F[15] * F[99];F[91] += F[17] * F[100];F[91] += F[19] * F[101];F[91] += F[21] * F[102];F[103] = (1 / (1 + Math.exp(-F[91])));F[104] = F[103] * (1 - F[103]);F[105] = F[103];F[106] = F[103];F[107] = F[103];F[108] = F[103];F[109] = F[103];
F[110] = F[111];F[111] = F[112];F[111] += F[3] * F[113];F[111] += F[5] * F[114];F[111] += F[7] * F[115];F[111] += F[9] * F[116];F[111] += F[11] * F[117];F[111] += F[13] * F[118];F[111] += F[15] * F[119];F[111] += F[17] * F[120];F[111] += F[19] * F[121];F[111] += F[21] * F[122];F[123] = (1 / (1 + Math.exp(-F[111])));F[124] = F[123] * (1 - F[123]);F[125] = F[123];
F[126] = F[127];F[127] = F[128];F[127] += F[3] * F[129];F[127] += F[5] * F[130];F[127] += F[7] * F[131];F[127] += F[9] * F[132];F[127] += F[11] * F[133];F[127] += F[13] * F[134];F[127] += F[15] * F[135];F[127] += F[17] * F[136];F[127] += F[19] * F[137];F[127] += F[21] * F[138];F[139] = (1 / (1 + Math.exp(-F[127])));F[140] = F[139] * (1 - F[139]);F[141] = F[139];
F[142] = F[143];F[143] = F[144];F[143] += F[3] * F[145];F[143] += F[5] * F[146];F[143] += F[7] * F[147];F[143] += F[9] * F[148];F[143] += F[11] * F[149];F[143] += F[13] * F[150];F[143] += F[15] * F[151];F[143] += F[17] * F[152];F[143] += F[19] * F[153];F[143] += F[21] * F[154];F[155] = (1 / (1 + Math.exp(-F[143])));F[156] = F[155] * (1 - F[155]);F[157] = F[155];
F[158] = F[159];F[159] = F[160];F[159] += F[3] * F[161];F[159] += F[5] * F[162];F[159] += F[7] * F[163];F[159] += F[9] * F[164];F[159] += F[11] * F[165];F[159] += F[13] * F[166];F[159] += F[15] * F[167];F[159] += F[17] * F[168];F[159] += F[19] * F[169];F[159] += F[21] * F[170];F[171] = (1 / (1 + Math.exp(-F[159])));F[172] = F[171] * (1 - F[171]);F[173] = F[171];
F[174] = F[175];F[175] = F[176];F[175] += F[3] * F[177];F[175] += F[5] * F[178];F[175] += F[7] * F[179];F[175] += F[9] * F[180];F[175] += F[11] * F[181];F[175] += F[13] * F[182];F[175] += F[15] * F[183];F[175] += F[17] * F[184];F[175] += F[19] * F[185];F[175] += F[21] * F[186];F[187] = (1 / (1 + Math.exp(-F[175])));F[188] = F[187] * (1 - F[187]);F[189] = F[187];
F[190] = F[191];F[191] = F[125] * F[192] * F[191] + F[193];F[191] += F[3] * F[194] * F[25];F[191] += F[5] * F[195] * F[26];F[191] += F[7] * F[196] * F[27];F[191] += F[9] * F[197] * F[28];F[191] += F[11] * F[198] * F[29];F[13] = (1 / (1 + Math.exp(-F[191])));F[199] = F[13] * (1 - F[13]);
F[200] = F[201];F[201] = F[141] * F[202] * F[201] + F[203];F[201] += F[3] * F[204] * F[45];F[201] += F[5] * F[205] * F[46];F[201] += F[7] * F[206] * F[47];F[201] += F[9] * F[207] * F[48];F[201] += F[11] * F[208] * F[49];F[15] = (1 / (1 + Math.exp(-F[201])));F[209] = F[15] * (1 - F[15]);
F[210] = F[211];F[211] = F[157] * F[212] * F[211] + F[213];F[211] += F[3] * F[214] * F[65];F[211] += F[5] * F[215] * F[66];F[211] += F[7] * F[216] * F[67];F[211] += F[9] * F[217] * F[68];F[211] += F[11] * F[218] * F[69];F[17] = (1 / (1 + Math.exp(-F[211])));F[219] = F[17] * (1 - F[17]);
F[220] = F[221];F[221] = F[173] * F[222] * F[221] + F[223];F[221] += F[3] * F[224] * F[85];F[221] += F[5] * F[225] * F[86];F[221] += F[7] * F[226] * F[87];F[221] += F[9] * F[227] * F[88];F[221] += F[11] * F[228] * F[89];F[19] = (1 / (1 + Math.exp(-F[221])));F[229] = F[19] * (1 - F[19]);
F[230] = F[231];F[231] = F[189] * F[232] * F[231] + F[233];F[231] += F[3] * F[234] * F[105];F[231] += F[5] * F[235] * F[106];F[231] += F[7] * F[236] * F[107];F[231] += F[9] * F[237] * F[108];F[231] += F[11] * F[238] * F[109];F[21] = (1 / (1 + Math.exp(-F[231])));F[239] = F[21] * (1 - F[21]);
F[240] = F[241];F[241] = F[242];F[241] += F[3] * F[243];F[241] += F[5] * F[244];F[241] += F[7] * F[245];F[241] += F[9] * F[246];F[241] += F[11] * F[247];F[241] += F[13] * F[248];F[241] += F[15] * F[249];F[241] += F[17] * F[250];F[241] += F[19] * F[251];F[241] += F[21] * F[252];F[253] = (1 / (1 + Math.exp(-F[241])));F[254] = F[253] * (1 - F[253]);F[255] = F[253];F[256] = F[253];F[257] = F[253];F[258] = F[253];
F[259] = F[260];F[260] = F[261];F[260] += F[3] * F[262];F[260] += F[5] * F[263];F[260] += F[7] * F[264];F[260] += F[9] * F[265];F[260] += F[11] * F[266];F[260] += F[13] * F[267];F[260] += F[15] * F[268];F[260] += F[17] * F[269];F[260] += F[19] * F[270];F[260] += F[21] * F[271];F[272] = (1 / (1 + Math.exp(-F[260])));F[273] = F[272] * (1 - F[272]);F[274] = F[272];F[275] = F[272];F[276] = F[272];F[277] = F[272];
F[278] = F[279];F[279] = F[280];F[279] += F[3] * F[281];F[279] += F[5] * F[282];F[279] += F[7] * F[283];F[279] += F[9] * F[284];F[279] += F[11] * F[285];F[279] += F[13] * F[286];F[279] += F[15] * F[287];F[279] += F[17] * F[288];F[279] += F[19] * F[289];F[279] += F[21] * F[290];F[291] = (1 / (1 + Math.exp(-F[279])));F[292] = F[291] * (1 - F[291]);F[293] = F[291];F[294] = F[291];F[295] = F[291];F[296] = F[291];
F[297] = F[298];F[298] = F[299];F[298] += F[3] * F[300];F[298] += F[5] * F[301];F[298] += F[7] * F[302];F[298] += F[9] * F[303];F[298] += F[11] * F[304];F[298] += F[13] * F[305];F[298] += F[15] * F[306];F[298] += F[17] * F[307];F[298] += F[19] * F[308];F[298] += F[21] * F[309];F[310] = (1 / (1 + Math.exp(-F[298])));F[311] = F[310] * (1 - F[310]);F[312] = F[310];F[313] = F[310];F[314] = F[310];F[315] = F[310];
F[316] = F[317];F[317] = F[318];F[317] += F[3] * F[319];F[317] += F[5] * F[320];F[317] += F[7] * F[321];F[317] += F[9] * F[322];F[317] += F[11] * F[323];F[317] += F[13] * F[324];F[317] += F[15] * F[325];F[317] += F[17] * F[326];F[317] += F[19] * F[327];F[317] += F[21] * F[328];F[329] = (1 / (1 + Math.exp(-F[317])));F[330] = F[329] * (1 - F[329]);F[331] = F[329];F[332] = F[329];F[333] = F[329];F[334] = F[329];
F[335] = F[336];F[336] = F[337];F[336] += F[3] * F[338];F[336] += F[5] * F[339];F[336] += F[7] * F[340];F[336] += F[9] * F[341];F[336] += F[11] * F[342];F[336] += F[13] * F[343];F[336] += F[15] * F[344];F[336] += F[17] * F[345];F[336] += F[19] * F[346];F[336] += F[21] * F[347];F[336] += F[348] * F[349];F[336] += F[350] * F[351];F[336] += F[352] * F[353];F[336] += F[354] * F[355];F[356] = (1 / (1 + Math.exp(-F[336])));F[357] = F[356] * (1 - F[356]);F[358] = F[356];F[359] = F[356];F[360] = F[356];F[361] = F[356];F[362] = F[356];F[363] = F[356];F[364] = F[356];F[365] = F[356];F[366] = F[356];F[367] = F[356];
F[368] = F[369];F[369] = F[370];F[369] += F[3] * F[371];F[369] += F[5] * F[372];F[369] += F[7] * F[373];F[369] += F[9] * F[374];F[369] += F[11] * F[375];F[369] += F[13] * F[376];F[369] += F[15] * F[377];F[369] += F[17] * F[378];F[369] += F[19] * F[379];F[369] += F[21] * F[380];F[369] += F[348] * F[381];F[369] += F[350] * F[382];F[369] += F[352] * F[383];F[369] += F[354] * F[384];F[385] = (1 / (1 + Math.exp(-F[369])));F[386] = F[385] * (1 - F[385]);F[387] = F[385];F[388] = F[385];F[389] = F[385];F[390] = F[385];F[391] = F[385];F[392] = F[385];F[393] = F[385];F[394] = F[385];F[395] = F[385];F[396] = F[385];
F[397] = F[398];F[398] = F[399];F[398] += F[3] * F[400];F[398] += F[5] * F[401];F[398] += F[7] * F[402];F[398] += F[9] * F[403];F[398] += F[11] * F[404];F[398] += F[13] * F[405];F[398] += F[15] * F[406];F[398] += F[17] * F[407];F[398] += F[19] * F[408];F[398] += F[21] * F[409];F[398] += F[348] * F[410];F[398] += F[350] * F[411];F[398] += F[352] * F[412];F[398] += F[354] * F[413];F[414] = (1 / (1 + Math.exp(-F[398])));F[415] = F[414] * (1 - F[414]);F[416] = F[414];F[417] = F[414];F[418] = F[414];F[419] = F[414];F[420] = F[414];F[421] = F[414];F[422] = F[414];F[423] = F[414];F[424] = F[414];F[425] = F[414];
F[426] = F[427];F[427] = F[428];F[427] += F[3] * F[429];F[427] += F[5] * F[430];F[427] += F[7] * F[431];F[427] += F[9] * F[432];F[427] += F[11] * F[433];F[427] += F[13] * F[434];F[427] += F[15] * F[435];F[427] += F[17] * F[436];F[427] += F[19] * F[437];F[427] += F[21] * F[438];F[427] += F[348] * F[439];F[427] += F[350] * F[440];F[427] += F[352] * F[441];F[427] += F[354] * F[442];F[443] = (1 / (1 + Math.exp(-F[427])));F[444] = F[443] * (1 - F[443]);F[445] = F[443];F[446] = F[443];F[447] = F[443];F[448] = F[443];F[449] = F[443];F[450] = F[443];F[451] = F[443];F[452] = F[443];F[453] = F[443];F[454] = F[443];
F[455] = F[456];F[456] = F[457];F[456] += F[3] * F[458];F[456] += F[5] * F[459];F[456] += F[7] * F[460];F[456] += F[9] * F[461];F[456] += F[11] * F[462];F[456] += F[13] * F[463];F[456] += F[15] * F[464];F[456] += F[17] * F[465];F[456] += F[19] * F[466];F[456] += F[21] * F[467];F[456] += F[348] * F[468];F[456] += F[350] * F[469];F[456] += F[352] * F[470];F[456] += F[354] * F[471];F[472] = (1 / (1 + Math.exp(-F[456])));F[473] = F[472] * (1 - F[472]);F[474] = F[472];
F[475] = F[476];F[476] = F[477];F[476] += F[3] * F[478];F[476] += F[5] * F[479];F[476] += F[7] * F[480];F[476] += F[9] * F[481];F[476] += F[11] * F[482];F[476] += F[13] * F[483];F[476] += F[15] * F[484];F[476] += F[17] * F[485];F[476] += F[19] * F[486];F[476] += F[21] * F[487];F[476] += F[348] * F[488];F[476] += F[350] * F[489];F[476] += F[352] * F[490];F[476] += F[354] * F[491];F[492] = (1 / (1 + Math.exp(-F[476])));F[493] = F[492] * (1 - F[492]);F[494] = F[492];
F[495] = F[496];F[496] = F[497];F[496] += F[3] * F[498];F[496] += F[5] * F[499];F[496] += F[7] * F[500];F[496] += F[9] * F[501];F[496] += F[11] * F[502];F[496] += F[13] * F[503];F[496] += F[15] * F[504];F[496] += F[17] * F[505];F[496] += F[19] * F[506];F[496] += F[21] * F[507];F[496] += F[348] * F[508];F[496] += F[350] * F[509];F[496] += F[352] * F[510];F[496] += F[354] * F[511];F[512] = (1 / (1 + Math.exp(-F[496])));F[513] = F[512] * (1 - F[512]);F[514] = F[512];
F[515] = F[516];F[516] = F[517];F[516] += F[3] * F[518];F[516] += F[5] * F[519];F[516] += F[7] * F[520];F[516] += F[9] * F[521];F[516] += F[11] * F[522];F[516] += F[13] * F[523];F[516] += F[15] * F[524];F[516] += F[17] * F[525];F[516] += F[19] * F[526];F[516] += F[21] * F[527];F[516] += F[348] * F[528];F[516] += F[350] * F[529];F[516] += F[352] * F[530];F[516] += F[354] * F[531];F[532] = (1 / (1 + Math.exp(-F[516])));F[533] = F[532] * (1 - F[532]);F[534] = F[532];
F[535] = F[536];F[536] = F[474] * F[537] * F[536] + F[538];F[536] += F[3] * F[539] * F[358];F[536] += F[5] * F[540] * F[359];F[536] += F[7] * F[541] * F[360];F[536] += F[9] * F[542] * F[361];F[536] += F[11] * F[543] * F[362];F[536] += F[13] * F[544] * F[363];F[536] += F[15] * F[545] * F[364];F[536] += F[17] * F[546] * F[365];F[536] += F[19] * F[547] * F[366];F[536] += F[21] * F[548] * F[367];F[348] = (1 / (1 + Math.exp(-F[536])));F[549] = F[348] * (1 - F[348]);
F[550] = F[551];F[551] = F[494] * F[552] * F[551] + F[553];F[551] += F[3] * F[554] * F[387];F[551] += F[5] * F[555] * F[388];F[551] += F[7] * F[556] * F[389];F[551] += F[9] * F[557] * F[390];F[551] += F[11] * F[558] * F[391];F[551] += F[13] * F[559] * F[392];F[551] += F[15] * F[560] * F[393];F[551] += F[17] * F[561] * F[394];F[551] += F[19] * F[562] * F[395];F[551] += F[21] * F[563] * F[396];F[350] = (1 / (1 + Math.exp(-F[551])));F[564] = F[350] * (1 - F[350]);
F[565] = F[566];F[566] = F[514] * F[567] * F[566] + F[568];F[566] += F[3] * F[569] * F[416];F[566] += F[5] * F[570] * F[417];F[566] += F[7] * F[571] * F[418];F[566] += F[9] * F[572] * F[419];F[566] += F[11] * F[573] * F[420];F[566] += F[13] * F[574] * F[421];F[566] += F[15] * F[575] * F[422];F[566] += F[17] * F[576] * F[423];F[566] += F[19] * F[577] * F[424];F[566] += F[21] * F[578] * F[425];F[352] = (1 / (1 + Math.exp(-F[566])));F[579] = F[352] * (1 - F[352]);
F[580] = F[581];F[581] = F[534] * F[582] * F[581] + F[583];F[581] += F[3] * F[584] * F[445];F[581] += F[5] * F[585] * F[446];F[581] += F[7] * F[586] * F[447];F[581] += F[9] * F[587] * F[448];F[581] += F[11] * F[588] * F[449];F[581] += F[13] * F[589] * F[450];F[581] += F[15] * F[590] * F[451];F[581] += F[17] * F[591] * F[452];F[581] += F[19] * F[592] * F[453];F[581] += F[21] * F[593] * F[454];F[354] = (1 / (1 + Math.exp(-F[581])));F[594] = F[354] * (1 - F[354]);
F[595] = F[596];F[596] = F[597];F[596] += F[3] * F[598];F[596] += F[5] * F[599];F[596] += F[7] * F[600];F[596] += F[9] * F[601];F[596] += F[11] * F[602];F[596] += F[13] * F[603];F[596] += F[15] * F[604];F[596] += F[17] * F[605];F[596] += F[19] * F[606];F[596] += F[21] * F[607];F[596] += F[348] * F[608];F[596] += F[350] * F[609];F[596] += F[352] * F[610];F[596] += F[354] * F[611];F[612] = (1 / (1 + Math.exp(-F[596])));F[613] = F[612] * (1 - F[612]);F[614] = F[612];F[615] = F[612];F[616] = F[612];F[617] = F[612];
F[618] = F[619];F[619] = F[620];F[619] += F[3] * F[621];F[619] += F[5] * F[622];F[619] += F[7] * F[623];F[619] += F[9] * F[624];F[619] += F[11] * F[625];F[619] += F[13] * F[626];F[619] += F[15] * F[627];F[619] += F[17] * F[628];F[619] += F[19] * F[629];F[619] += F[21] * F[630];F[619] += F[348] * F[631];F[619] += F[350] * F[632];F[619] += F[352] * F[633];F[619] += F[354] * F[634];F[635] = (1 / (1 + Math.exp(-F[619])));F[636] = F[635] * (1 - F[635]);F[637] = F[635];F[638] = F[635];F[639] = F[635];F[640] = F[635];
F[641] = F[642];F[642] = F[643];F[642] += F[3] * F[644];F[642] += F[5] * F[645];F[642] += F[7] * F[646];F[642] += F[9] * F[647];F[642] += F[11] * F[648];F[642] += F[13] * F[649];F[642] += F[15] * F[650];F[642] += F[17] * F[651];F[642] += F[19] * F[652];F[642] += F[21] * F[653];F[642] += F[348] * F[654];F[642] += F[350] * F[655];F[642] += F[352] * F[656];F[642] += F[354] * F[657];F[658] = (1 / (1 + Math.exp(-F[642])));F[659] = F[658] * (1 - F[658]);F[660] = F[658];F[661] = F[658];F[662] = F[658];F[663] = F[658];
F[664] = F[665];F[665] = F[666];F[665] += F[3] * F[667];F[665] += F[5] * F[668];F[665] += F[7] * F[669];F[665] += F[9] * F[670];F[665] += F[11] * F[671];F[665] += F[13] * F[672];F[665] += F[15] * F[673];F[665] += F[17] * F[674];F[665] += F[19] * F[675];F[665] += F[21] * F[676];F[665] += F[348] * F[677];F[665] += F[350] * F[678];F[665] += F[352] * F[679];F[665] += F[354] * F[680];F[681] = (1 / (1 + Math.exp(-F[665])));F[682] = F[681] * (1 - F[681]);F[683] = F[681];F[684] = F[681];F[685] = F[681];F[686] = F[681];
F[687] = F[688];F[688] = F[689];F[688] += F[13] * F[690] * F[255];F[688] += F[15] * F[691] * F[274];F[688] += F[17] * F[692] * F[293];F[688] += F[19] * F[693] * F[312];F[688] += F[21] * F[694] * F[331];F[688] += F[348] * F[695] * F[614];F[688] += F[350] * F[696] * F[637];F[688] += F[352] * F[697] * F[660];F[688] += F[354] * F[698] * F[683];F[688] += F[3] * F[699];F[688] += F[5] * F[700];F[688] += F[7] * F[701];F[688] += F[9] * F[702];F[688] += F[11] * F[703];F[704] = (1 / (1 + Math.exp(-F[688])));F[705] = F[704] * (1 - F[704]);
F[706] = F[707];F[707] = F[708];F[707] += F[13] * F[709] * F[256];F[707] += F[15] * F[710] * F[275];F[707] += F[17] * F[711] * F[294];F[707] += F[19] * F[712] * F[313];F[707] += F[21] * F[713] * F[332];F[707] += F[348] * F[714] * F[615];F[707] += F[350] * F[715] * F[638];F[707] += F[352] * F[716] * F[661];F[707] += F[354] * F[717] * F[684];F[707] += F[3] * F[718];F[707] += F[5] * F[719];F[707] += F[7] * F[720];F[707] += F[9] * F[721];F[707] += F[11] * F[722];F[723] = (1 / (1 + Math.exp(-F[707])));F[724] = F[723] * (1 - F[723]);
F[725] = F[726];F[726] = F[727];F[726] += F[13] * F[728] * F[257];F[726] += F[15] * F[729] * F[276];F[726] += F[17] * F[730] * F[295];F[726] += F[19] * F[731] * F[314];F[726] += F[21] * F[732] * F[333];F[726] += F[348] * F[733] * F[616];F[726] += F[350] * F[734] * F[639];F[726] += F[352] * F[735] * F[662];F[726] += F[354] * F[736] * F[685];F[726] += F[3] * F[737];F[726] += F[5] * F[738];F[726] += F[7] * F[739];F[726] += F[9] * F[740];F[726] += F[11] * F[741];F[742] = (1 / (1 + Math.exp(-F[726])));F[743] = F[742] * (1 - F[742]);
F[744] = F[745];F[745] = F[746];F[745] += F[13] * F[747] * F[258];F[745] += F[15] * F[748] * F[277];F[745] += F[17] * F[749] * F[296];F[745] += F[19] * F[750] * F[315];F[745] += F[21] * F[751] * F[334];F[745] += F[348] * F[752] * F[617];F[745] += F[350] * F[753] * F[640];F[745] += F[352] * F[754] * F[663];F[745] += F[354] * F[755] * F[686];F[745] += F[3] * F[756];F[745] += F[5] * F[757];F[745] += F[7] * F[758];F[745] += F[9] * F[759];F[745] += F[11] * F[760];F[761] = (1 / (1 + Math.exp(-F[745])));F[762] = F[761] * (1 - F[761]);
var output = [];
output[0] = F[704];
output[1] = F[723];
output[2] = F[742];
output[3] = F[761];
return output;
}