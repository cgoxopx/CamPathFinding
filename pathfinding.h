#ifndef IMG_PATHFINDIND
#define IMG_PATHFINDIND

typedef struct pfPoint{
    int x;
    int y;
}pfPoint_t;

typedef struct pfLine{
    float a,b;//y=ax+b
}pfLine_t;

typedef struct pfSeg{
    pfLine_t left;
    pfLine_t right;
    int leftPointNum;
    int rightPointNum;
    float leftBlackPrecent;
    float rightBlackPrecent;
    int leftLineExist;
    int rightLineExist;
    
    int from,to;
    
    float center;//左边与右边距离差，表示是否在路中间，小于0向右，大于0向左
    
    float leftAverage;
    float rightAverage;
    
}pfSeg_t;

typedef struct pathfinding{
    void * arg;
    
    char (*getImage)(void * arg,int x,int y);
    //get pix of input image
    //return 1:black or 0:white
    
    char (*getLine)(void * arg,int x,int y,int id);
    
    char (*getBuffer)(void * arg,int x,int y);
    //get pix of buffer
    //return 2:edge 1:black or 0:white
    
    void (*setBuffer)(void * arg,int x,int y,char pix);
    //set pix of buffer
    
    int H;//max y
    int W;//max x
    
    int   d;//投影点，一般是正中心y坐标
    int   h;//摄像机高度
    float sint;
    float cost;
    int   actLen;
    
    float persMat[16];//透视矩阵
    
    pfPoint_t * segBufferLeft;
    int segBufferLeftLen;
    pfPoint_t * segBufferRight;
    int segBufferRightLen;
    float a,b;//x=a*(-y)+b 颠倒x，y，防止斜率不存在
    /*
     * Y
     * |
     * |
     * |
     * |       |
     * |       |
     * |       |
     * |       |
     * |       |
     * O----------------X
     * 
     * 
     * X
     * |
     * |
     * |
     * |
     * |----------------
     * |
     * |
     * |
     * O-----------------(-Y)
    */
    
    pfSeg_t * segs;
    int segNum;
    
    float curveThres;//斜率与第一个seg相差多少判断为转弯
    //int segPointAverage;
    int cyThres;//左右点相差多少判断为环
    int lineThres;//点数比前后相差多少判断为横线
    float blackDelta;//黑色区域阈值
    float whiteDelta;//白色区域阈值
    
    int resultSegNum;
    
}pathfinding_t;


int pfLinearFit(pfPoint_t * buf,int len,pfLine_t * out,float * averageDelta);

void pfCreateSeg(
    pathfinding_t * self,
    int from,
    int to,
    pfLine_t * left,
    pfLine_t * right,
    int * lnum,
    int * rnum,
    int * lex,
    int * rex,
    int * lbn,
    int * rbn,
    int * lpn,
    int * rpn,
    float * lave,
    float * rave
);

void pfGetSegCenter(pfSeg_t * seg);

void pfSegAll(pathfinding_t * self,int segsize);

void pfPersMatInit(pathfinding_t * self);

void pfPointxMat4(const float * vec,const float * mat,float * out);

void pfVec4xMat4(const float * vec,const float * mat,float * out);

void pfVec42Vec3(const float * vec,float * out);

void pfFix(pathfinding_t * self,int con);

void getEdge(pathfinding_t * self);

void pfGetPersPosition(pathfinding_t * self,int x,int y,int * ox,int * oy);
//x y:position in map;
//ox oy:position in image

void pfGetUnpPosition(pathfinding_t * self,int x,int y,float * ox,float * oy);
//x y:positon in image
//ox oy:position in map

void pfUnpersDe(pathfinding_t * self);
void pfUnpers(pathfinding_t * self);
//unperspective the image

int pfHaveCrossInSi(pathfinding_t * self,int index);

int pfHaveCylInSi(pathfinding_t * self,int index,int cmp);
int pfHaveCylInSeg(pfSeg_t * seg,pfSeg_t * pseg);
//1:left 
//2:right
//3:both

float pfHaveCurveInSi(pathfinding_t * self,int index);
int pfHaveCurve(pathfinding_t * self);

int pfFilledWhite(pathfinding_t * self,int index);

int pfHaveBlack(pathfinding_t * self,int index);

int phHaveObst(pathfinding_t * self);

#endif
