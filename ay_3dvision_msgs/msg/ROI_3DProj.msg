uint8 NONE=0
uint8 POLYGON=1     #param= [x0,y0,z0,x1,y1,z1,...]
uint8 CONE=2        #param= [cx,cy,cz,ex,ey,ez,angle,length]  c:center, e:direction, angle:angle of half cone(i.e. bottom radius=length*tan(angle))

string target  #Target image; "col_det", "stereo_f", ...
int32 type
float64[] param
