/*
* dataType.cpp
* Created on: 20210609
* Author: yueyingying
* Description: 单个检测结果目标类
*/
#include "dataType.h"
#include <algorithm>
#include <iostream>

namespace camera
{

const float kRatio=0.5;
enum DETECTBOX_IDX {IDX_X = 0, IDX_Y, IDX_W, IDX_H };

// tlwh 转cx cy ration h
Bbox DetectionRow::ToCxcyah() const
{//(centerx, centery, ration, h)
	Bbox ret = tlwh;
	ret(0, IDX_X) += (ret(0, IDX_W)*kRatio);
	ret(0, IDX_Y) += (ret(0, IDX_H)*kRatio);
	ret(0, IDX_W) /= ret(0, IDX_H);
	return ret;
}

// tlwh 转cx cy w h
Bbox DetectionRow::ToCxcywh() const
{//(centerx, centery, ration, h)
	Bbox ret = tlwh;
	ret(0, IDX_X) += (ret(0, IDX_W)*kRatio);
	ret(0, IDX_Y) += (ret(0, IDX_H)*kRatio);
	// ret(0, IDX_W) /= ret(0, IDX_H);
	return ret;
}

// tlwh 转 xmin,ymin,xmax,ymax
Bbox DetectionRow::ToTlbr() const
{
	Bbox ret = tlwh;
	ret(0, IDX_X) += ret(0, IDX_W);
	ret(0, IDX_Y) += ret(0, IDX_H);
	return ret;
}

Bbox DetectionRow::ToCxymax() const
{//(cx,ymax,w,h)
	Bbox ret = tlwh;
	ret(0, IDX_X) += (ret(0, IDX_W)*kRatio);
	ret(0, IDX_Y) += ret(0, IDX_H);
	return ret;
}

TrackNode::TrackNode()
{
	this->lane_idx = -1;
	this->link_idx = -1;
	this->confidence = 0;
	this->longitude = 0;
	this->latitude = 0;
    this->plate = "0"; //plate
	this->plate_conf = 0;
	this->speed = 0;
	this->id = -1;
	this->acceleration = 0;
	this->confirmed = false;

	this->link_heading = -1;
	this->appear_area = -1;
	// this->last_heading = -1;

	this->state = e_State::NORMAL;
}

void TrackNode::Init(int track_id, int type, Bbox detection, float heading, u_int64_t timestamp)
{
	this->cywh = detection;
	this->heading = heading;
	this->speed = 0;
	this->timestamp = timestamp;
	this->type = type;
	this->id = track_id;

	this->lane_idx = -1;
	this->link_idx = -1;
	this->confidence = 0;
	this->longitude = 0;
	this->latitude = 0;
    this->plate = "0"; //plate
	this->plate_conf = 0;
}

void TrackNode::Print()
{	
	printf("%d (%.1f, %.1f, %.1f, %.1f) %ld %.2f %.2f %.6f %.6f\n", id, cywh[0], cywh[1], cywh[2], cywh[3],
			timestamp, speed, heading, longitude, latitude);
}

}
