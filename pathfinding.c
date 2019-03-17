#include "pathfinding.h"

void pfGetUnpPosition(pathfinding_t * self,int x,int y,float * ox,float * oy){
    *oy=(self->sint*y - self->h)/(self->cost+(self->h/self->d));
    *ox=(x*(*oy + self->d))/self->d;
}

void pfVec4xMat4(const float * vec,const float * mat,float * out){
    /*
     * |0   1   2   3 |
     * |              |
     * |4   5   6   7 |
     * |              |
     * |8   9   10  11|
     * |              |
     * |12  13  14  15|
     * 
     */
    out[0]=vec[0]*mat[0] + vec[1]*mat[4] + vec[2]*mat[8]  + vec[3]*mat[12];
    out[1]=vec[0]*mat[1] + vec[1]*mat[5] + vec[2]*mat[9]  + vec[3]*mat[13];
    out[2]=vec[0]*mat[2] + vec[1]*mat[6] + vec[2]*mat[10] + vec[3]*mat[14];
    out[3]=vec[0]*mat[3] + vec[1]*mat[7] + vec[2]*mat[11] + vec[3]*mat[15];
}

void pfVec42Vec3(const float * vec,float * out){
    out[0]=vec[0]/vec[3];
    out[1]=vec[1]/vec[3];
    out[2]=vec[2]/vec[3];
}


void pfPointxMat4(const float * vec,const float * mat,float * out){
    float vec4[4];
    float buf[4];
    vec4[0]=vec[0];
    vec4[1]=vec[1];
    vec4[2]=vec[2];
    vec4[3]=1;
    pfVec4xMat4(vec4,mat,buf);
    pfVec42Vec3(buf,out);
}

void pfPersMatInit(pathfinding_t * self){
    /*
     * 
     *              |1       0       0       0           |
     *              |                                    |
     *              |0       cost    0       -sint/d     |
     * (x,y,0,1)  * |                                    |
     *              |0       -sint   0       -cost/d     |
     *              |                                    |
     *              |0       h*sint  0       1+(h*cost)/d|
     * 
     */
    self->persMat[0] =1;
    self->persMat[1] =0;
    self->persMat[2] =0;
    self->persMat[3] =0;
    self->persMat[4] =0;
    self->persMat[5] =self->cost;
    self->persMat[6] =0;
    self->persMat[7] =(-(self->sint))/self->d;
    self->persMat[8] =0;
    self->persMat[9] =-(self->sint);
    self->persMat[10]=0;
    self->persMat[11]=(-(self->cost))/self->d;
    self->persMat[12]=0;
    self->persMat[13]=self->h*self->sint;
    self->persMat[14]=0;
    self->persMat[15]=(1+(self->h*self->cost))/self->d;
}
void pfGetPersPosition(pathfinding_t * self,int x,int y,int * ox,int * oy){
    float point[3]={x,y,0};
    float buf[3];
    pfPointxMat4(point,self->persMat,buf);
    *ox=buf[0];
    *oy=buf[1];
}

void pfUnpersDe(pathfinding_t * self){
    pfPersMatInit(self);
    int i,j;
    int ox,oy;
    for(i=0;i<self->W;++i){//x
        for(j=0;j<self->H;++j){//y
            pfGetPersPosition(self,i,j,&ox,&oy);
            if(ox<0 || ox>=self->W || oy<0 || oy>=self->H){
                self->setBuffer(self,i,j,0);
                continue;
            }
            if(self->getImage(self->arg,ox,oy)!=0){
                self->setBuffer(self,i,j,1);
            }else{
                self->setBuffer(self,i,j,0);
            }
        }
    }
}
void pfUnpers(pathfinding_t * self){
    //会大量重复调用pfUnpers，建议如果芯片有gpu，请直接使用gpu
    int i,j;
    float ox,oy;
    int iox,ioy;
    for(i=0;i<self->W;++i){//x
        for(j=0;j<self->H;++j){//y
            if(self->getImage(self->arg,i,j)!=0){
                pfGetUnpPosition(self,i,j,&ox,&oy);
                iox=ox;
                ioy=oy;
                if(iox<0 || iox>=self->W || ioy<0 || ioy>=self->H){
                    continue;
                }
                self->setBuffer(self,iox,ioy,1);
            }
        }
    }
}

int pfLinearFit(pfPoint_t * buf,int len,pfLine_t * out){
    int i;
    int sx=0,sy=0;
    float ax,ay;
    float sup=0,sdown=0;
    
    if(len<=0)
        return 0;
    for(i=0;i<len;++i){
        sx+=buf[i].x;
        sy+=buf[i].y;
    }
    //get average
    ax=((float)sx)/((float)len);
    ay=((float)sy)/((float)len);
    
    for(i=0;i<len;++i){
        sup     +=(buf[i].x-ax)*(buf[i].y-ay);
        sdown   +=(buf[i].x-ax)*(buf[i].x-ax);
    }
    if(sdown==0)
        return 0;
    out->a = sup / sdown;
    out->b = ay - out->a * ax;
    return 1;
}

void pfCreateSeg(
    pathfinding_t * self,
    int from,
    int to,
    pfLine_t * left,
    pfLine_t * right,
    int * lnum,
    int * rnum,
    int * lex,
    int * rex
){
    int tx,ty,ly,delta;
    int i,j;
    int il=0,ir=0;
    for(i=0;i<self->W;++i){//x
        for(j=from;j<=to;++j){//y
            if(self->getBuffer(self->arg,i,j)!=0){
                tx=-j;
                ty= i;
                ly=tx*self->a+self->b;
                delta=ty-ly;
                if(delta>0){//left
                    if(il < self->segBufferLeftLen){
                        self->segBufferLeft[il].x=tx;
                        self->segBufferLeft[il].y=delta;
                        ++il;
                    }
                }else{//right
                    if(ir < self->segBufferRightLen){
                        self->segBufferRight[ir].x=tx;
                        self->segBufferRight[ir].y=delta;
                        ++ir;
                    }
                }
            }
        }
    }
    
    *lex=0;
    *rex=0;
    
    *lnum=il;
    if(il>0){
        *lex=pfLinearFit(self->segBufferLeft,il,left);
    }
    
    *rnum=ir;
    if(ir>0){
        *rex=pfLinearFit(self->segBufferRight,ir,right);
    }
}

void pfSegAll(pathfinding_t * self,int segsize){
    int i=0;
    int to;
    int index=0;
    while(1){
        
        if(i >= self->H-1)
            break;
        
        to=i+segsize;
        
        if(to>=self->H-1)
            to=self->H-1;
        
        if(index >= self->segNum)
            break;
        
        pfCreateSeg(
            self,i,to,
            &(self->segs[index].left),
            &(self->segs[index].right),
            &(self->segs[index].leftPointNum),
            &(self->segs[index].rightPointNum),
            &(self->segs[index].leftLineExist),
            &(self->segs[index].rightLineExist)
        );
        
        i=i+segsize+1;
    }
}
void pfGetSegCenter(pfSeg_t * seg){
    float lt,rt;
    
}

int pfHaveForkInSi(pathfinding_t * self,int * len){
    
}
int pfHaveFork(pathfinding_t * self){
    
}

int pfHaveCylInSi(pathfinding_t * self,int * len){
    
}
int pfHaveCyl(pathfinding_t * self){
    
}

int pfHaveCurveInSi(pathfinding_t * self,int * len){
    
}
int pfHaveCurve(pathfinding_t * self){
    
}