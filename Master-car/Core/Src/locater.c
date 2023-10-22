#include "locater.h"

locater_def locater = {.pos_x_base=675-24,.pos_y_base=75.5f};

pointStruct spot[4] = {0};
uint8_t spotNum=0;

pointStruct startSpot[4] = {{871,555.5f,90},
                       {1101,555.5f,90},
                       {1101,95.5f,45},
                       {651,75.5f,0}};
uint8_t startSpotNum=0;


const pointStruct ROUTE_PIONT_RING_FRONT[3] = {{561,385,180},
                                               {600,385,180},
                                               {638,385,180}};


const pointStruct POINT_TAKE_RING[3] = {{561,412,180},
                                        {600,412,180},
                                        {638,412,180}};

const pointStruct ROUTE_COLUMN[15] = {{600,378,0},
                                      {600,378,104.4f},
                                      {600,378,180},
                                      {600,378,-105.6f},
                                      {600,378,-90},
                                      {600,378,90},
                                      {600,378,19.45f},
                                      {600,378,-19.91f},
                                      {600,378,0},
                                      {600,378,51.9f},
                                      {600,378,-51.9f},
                                      {600,378,150},
                                      {600,378,-150},
                                      {600,378,-135},
                                      {600,378,135}};

PID_TypeDef PID_route_angle,PID_route_x,PID_route_y;

void locater_data_rec(uint8_t *data, locater_def *loc)
{
    loc_receive_union union_loc;
    if(data[0] == 'P' && data[1] == 'G')
    {
        loc->LastlastAngle = loc->lastAngle;

        loc->pos_x_last = loc->pos_x;
        loc->pos_y_last = loc->pos_y;
        loc->lastAngle = loc->angle;

        for(int i = 0; i < 12; i++)
        {
            union_loc.data_8[i] = data[i+2];
        }

        loc->pos_x = 1.0054746843f*union_loc.data_f[0] + loc->pos_x_base;
        loc->pos_y = 1.0054746843f*union_loc.data_f[1] + loc->pos_y_base;
        loc->angle = union_loc.data_f[2];

        loc->speed_x = 500*(loc->pos_x - loc->pos_x_last);
        loc->speed_y = 500*(loc->pos_y - loc->pos_y_last);
        loc->angular_speed = 500*(loc->angle - loc->lastAngle);

        if((loc->angle - loc->lastAngle) < -180.0f)
        {
            loc->circleNum++;
        }
        else if((loc->angle - loc->lastAngle) > 180.0f)
        {
            loc->circleNum--;
        }
        loc->continuousAngle = (float)loc->circleNum * 360.0f + loc->angle;

    }
}

pointStruct RouteInsertLinear(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination)
{
    float t = (float)((float)counting / (float)total);
    float k = (float)(((float)total - (float)counting) / (float)total);
    pointStruct outputPiont;
    outputPiont.x = startingPiont->x * k + destination->x * t;
    outputPiont.y = startingPiont->y * k + destination->y * t;
    outputPiont.angle = startingPiont->angle * k + destination->angle * t;
    return outputPiont;
}

pointStruct RouteInsertBezier_2(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination, pointStruct* crolPiont)
{
    pointStruct outputPiont;
    float t = (float)((float)counting / (float)total);
    float k = (float)(((float)total - (float)counting) / (float)total);
    outputPiont.x = startingPiont->x * k * k + crolPiont->x * 2 * t * k + destination->x * t * t;
    outputPiont.y = startingPiont->y * k * k + crolPiont->y * 2 * t * k + destination->y * t * t;
    outputPiont.angle = startingPiont->angle * k * k + crolPiont->angle * 2 * t * k + destination->angle * t * t;
    return outputPiont;
}

pointStruct RouteInsertBezier_3(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination, pointStruct* crolPiont_1, pointStruct* crolPiont_2)
{
    pointStruct outputPiont;
    float t = (float)((float)counting / (float)total);
    float k = (float)(((float)total - (float)counting) / (float)total);
    outputPiont.x = startingPiont->x * k * k * k + crolPiont_1->x * 3 * t * k * k + crolPiont_2->x * 3 * t * t * k + destination->x * t * t * t;
    outputPiont.y = startingPiont->y * k * k * k + crolPiont_1->y * 3 * t * k * k + crolPiont_2->y * 3 * t * t * k + destination->y * t * t * t;
    outputPiont.angle = startingPiont->angle * k * k * k + crolPiont_1->angle * 3 * t * k * k + crolPiont_2->angle * 3  * t * t * k + destination->angle * t * t * t;
    return outputPiont;
}

float distanceCal(pointStruct point, float x, float y)
{
    return sqrtf((point.x - x) * (point.x - x) + (point.y - y) * (point.y - y));
}


float routeDistanceCal(pointStruct* point1, pointStruct* point2)
{
    return sqrtf((point1->x - point2->x) * (point1->x - point2->x) + (point1->y - point2->y) * (point1->y - point2->y));
}

float RouteAngleOptimize(float targetAngle,float presentAngle)
{
    float differAngle;
    differAngle = targetAngle - presentAngle;
    while(differAngle >= 180.0f)
    {
        differAngle -= 360.0f;
    }
    while(differAngle < -180.0f)
    {
        differAngle += 360.0f;
    }
    return presentAngle + differAngle;
}


uint8_t AreaRecognize(float loc_x,float loc_y)
{
    uint8_t posFlag = 0;
    if(loc_x > X1 && loc_x < X2)
    {
        posFlag = 0;
    }
    else if(loc_x > X2 && loc_x < X3)
    {
        posFlag = 3;
    }
    else if(loc_x > X3 && loc_x < X4)
    {
        posFlag = 6;
    }
    else if(loc_x > X4 && loc_x < X5)
    {
        posFlag = 9;
    }
    else if(loc_x > X5 && loc_x < X6)
    {
        posFlag = 12;
    }

    if(loc_y > Y0 && loc_y < Y2)
    {
        posFlag += 0;
    }
    else if(loc_y > Y2 && loc_y < Y3)
    {
        posFlag += 1;
    }
    else if(loc_y > Y3 && loc_y < Y4)
    {
        posFlag += 2;
    }
    return posFlag;
}

void RouteFormulate(pointStruct destination,float loc_x,float loc_y,float loc_angle)
{
    uint8_t posFlag = 0;
    uint8_t routeCase = 0;

    posFlag = AreaRecognize(loc_x, loc_y);

    switch(posFlag)
    {
        case 0:
            if( (destination.x < (X3-RR_WIDE)) || (destination.y < (Y2-RR_WIDE)) )
                routeCase = 1;
            else
                routeCase = 2;
            break;
        case 1:
            if(destination.x < (X3-RR_WIDE))
                routeCase = 1;
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 2:
            if(destination.x < (X2-RR_WIDE))
                routeCase = 1;
            else if(destination.x < (X4-RR_WIDE))
                routeCase = 2;
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 3:
            if(destination.y < (Y2-RR_WIDE))
                routeCase = 1;
            else if(destination.y < (Y3-RR_WIDE))
            {
                if(destination.x < (X3-RR_WIDE))
                    routeCase = 1;
                else
                    routeCase = 2;
            }
            else
            {
                if(destination.x < (X2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 4:
            if(destination.x < (X3-RR_WIDE))
            {
                if(destination.y < (Y3-RR_WIDE))
                    routeCase = 1;
                else
                    routeCase = 2;
            }
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 6:
            if(destination.y < (Y2-RR_WIDE))
                routeCase = 1;
            else
                routeCase = 2;
            break;
        case 7:
            if(destination.y < (Y3-RR_WIDE))
                routeCase = 1;
            else
                routeCase = 2;
            break;
        case 9:
            if(destination.y < (Y2-RR_WIDE))
                routeCase = 1;
            else if(destination.y < (Y3-RR_WIDE))
            {
                if(destination.x > (X4+RR_WIDE))
                    routeCase = 1;
                else
                    routeCase = 2;
            }
            else
            {
                if(destination.x > (X5+RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 10:
            if(destination.x > (X4+RR_WIDE))
            {
                if(destination.y < (Y3-RR_WIDE))
                    routeCase = 1;
                else
                    routeCase = 2;
            }
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 12:
            if( (destination.x > (X4+RR_WIDE)) || (destination.y < (Y2-RR_WIDE)) )
                routeCase = 1;
            else
                routeCase = 2;
            break;
        case 13:
            if(destination.x > (X4+RR_WIDE))
                routeCase = 1;
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
        case 14:
            if(destination.x > (X5+RR_WIDE))
                routeCase = 1;
            else if(destination.x > (X3+RR_WIDE))
                routeCase = 2;
            else
            {
                if(destination.y < (Y2-RR_WIDE))
                    routeCase = 2;
                else
                    routeCase = 3;
            }
            break;
    }

    switch (routeCase)
    {
        case 1:
            spot[0] = destination;

            spot[1].x = loc_x;
            spot[1].y = loc_y;
            spot[1].angle = loc_angle;
            break;
        case 2:
            spot[0] = destination;
            if(loc_y < destination.y)
            {
                spot[1].y = loc_y;
                spot[1].x = destination.x;
            }
            else
            {
                spot[1].y = destination.y;
                spot[1].x = loc_x;
            }
            spot[1].angle = (destination.angle + loc_angle)/2.0f;

            spot[2].x = loc_x;
            spot[2].y = loc_y;
            spot[2].angle = loc_angle;

            break;
        case 3:
            spot[0] = destination;

            spot[1].y = Y_ROUTE;
            spot[1].x = destination.x;
            spot[1].angle = 0;

            spot[2].y = Y_ROUTE;
            spot[2].x = loc_x;
            spot[2].angle = 0;

            spot[3].x = loc_x;
            spot[3].y = loc_y;
            spot[3].angle = loc_angle;

            break;
    }
    spotNum = routeCase;
}

uint8_t AutoRouteMotive(pointStruct* route,uint8_t* num)
{
    static int32_t counting,countingMax;
    static pointStruct ctrlPoint;
    static uint8_t changeFlag = 1;
    static float xSpeed,ySpeed,angularSpeed;
    if(*num > 0)
    {
        if(changeFlag)
        {
            counting = 0;
            countingMax = 1;//(int)routeDistanceCal((route + *num),(route + *num - 1)) / 2;
            ctrlPoint = RouteInsertLinear(counting, countingMax, (route + *num), (route + *num - 1));

            ctrlPoint.angle = RouteAngleOptimize(ctrlPoint.angle, locater.continuousAngle);

            changeFlag = 0;
        }
        else
        {
            if(counting < countingMax)
            {
                if(distanceCal(ctrlPoint,locater.pos_x,locater.pos_y) < 7)
                {
                    counting++;

                    ctrlPoint = RouteInsertLinear(counting, countingMax, (route + *num), (route + *num - 1));

                    ctrlPoint.angle = RouteAngleOptimize(ctrlPoint.angle, locater.continuousAngle);

                }
            }
            else
            {
                *num = *num -1;
                changeFlag = 1;
            }
        }
        xSpeed = (float)pid_calculate(&PID_route_x, locater.pos_x * 100.0f,ctrlPoint.x * 100.0f);
        ySpeed = (float)pid_calculate(&PID_route_y, locater.pos_y * 100.0f,ctrlPoint.y * 100.0f);
        angularSpeed = (float)pid_calculate(&PID_route_angle, locater.continuousAngle * 100.0f,ctrlPoint.angle * 100.0f);
        SetRobotVel_XY(&rrWheel, xSpeed, ySpeed, angularSpeed, locater.angle);
        //usart_printf("%f,%f,%f\n",ctrlPoint.x,ctrlPoint.y,ctrlPoint.angle);
    }

    //usart_printf("%f\n",ctrlPoint.angle,locater.continuousAngle);

    return *num;
}

