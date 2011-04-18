typedef union {
  double dval; int ival;
} YYSTYPE;
#define NUM 258
#define STR 259
#define GE  260
#define LE  261
#define UE  262
#define ID  263
#define EQEQ    264
#define ASSIGN  265
#define PRINT   266
#define IF  267
#define DO  268
#define NEWLINE 269
#define THEN    270
#define ELSE    271
#define WHILE   272
#define ENDIF   273
#define ENDWHILE    274
#define RUN 275
#define POSROT  276
#define SCRIPT  277
#define TORQUE  278
#define ABS 279
#define SIN 280
#define COS 281
#define ASIN    282
#define ACOS    283
#define TAN 284
#define ATAN    285
#define SLEEP   286
#define SENSOR  287
#define STOP    288
#define WAIT    289
#define MOTION  290
#define PLAY    291
#define LOAD    292
#define POS 293
#define ARML    294
#define ARMR    295
#define LEGL    296
#define LEGR    297
#define MOVETO  298
#define COLLISION   299
#define TIME    300
#define SYNC    301
#define ENDSYNC 302
#define HOLD    303
#define GET 304
#define SET 305
#define UMINUS  306


extern YYSTYPE yylval;