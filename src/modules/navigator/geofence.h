/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file geofence.h
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef GEOFENCE_H_
#define GEOFENCE_H_

#include <cfloat>

#include <controllib/block/BlockParam.hpp>
#include <controllib/blocks.hpp>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>

#define GEOFENCE_FILENAME PX4_ROOTFSDIR"/fs/microsd/etc/geofence.txt"

#define GEOFENCE_EWT PX4_ROOTFSDIR"/etc/extras/ewt.dat"

class Navigator;

class Geofence : public control::SuperBlock
{
public:
	Geofence(Navigator *navigator);
	Geofence(const Geofence &) = delete;
	Geofence &operator=(const Geofence &) = delete;
	~Geofence() = default;

	/* Altitude mode, corresponding to the param GF_ALTMODE */
	enum {
		GF_ALT_MODE_WGS84 = 0,
		GF_ALT_MODE_AMSL = 1
	};

	/* Source, corresponding to the param GF_SOURCE */
	enum {
		GF_SOURCE_GLOBALPOS = 0,
		GF_SOURCE_GPS = 1
	};

	/**
	 * Return whether system is inside geofence.
	 *
	 * Calculate whether point is inside arbitrary polygon
	 * @param craft pointer craft coordinates
	 * @return true: system is inside fence, false: system is outside fence
	 */
	bool inside(const struct vehicle_global_position_s &global_position,
		    const struct vehicle_gps_position_s &gps_position, float baro_altitude_amsl);

	bool inside(const struct mission_item_s &mission_item);

	bool inside_polygon(double lat, double lon, float altitude);

	int clearDm();

	bool valid();

	/**
	 * Specify fence vertex position.
	 */
	void addPoint(int argc, char *argv[]);

	void publishFence(unsigned vertices);

	int loadFromFile(const char *filename);
	int loadFromEwtFile(const struct vehicle_global_position_s &global_position);
	bool intsideEwtFile(const struct vehicle_global_position_s &global_position);

	bool isEmpty() {return _vertices_count == 0;}

	int getAltitudeMode() { return _param_altitude_mode.get(); }
	int getSource() { return _param_source.get(); }
	int getGeofenceAction() { return _param_action.get(); }

	bool isHomeRequired();

private:
	Navigator	*_navigator{nullptr};

	orb_advert_t	_fence_pub{nullptr};			/**< publish fence topic */

	hrt_abstime _last_horizontal_range_warning{0};
	hrt_abstime _last_vertical_range_warning{0};

	float _altitude_min{0.0f};
	float _altitude_max{0.0f};

	unsigned _vertices_count{0};

	/* Params */
	control::BlockParamInt _param_action;
	control::BlockParamInt _param_altitude_mode;
	control::BlockParamInt _param_source;
	control::BlockParamInt _param_counter_threshold;
	control::BlockParamFloat _param_max_hor_distance;
	control::BlockParamFloat _param_max_ver_distance;

	int _outside_counter{0};

	bool inside(double lat, double lon, float altitude);
	bool inside(const struct vehicle_global_position_s &global_position);
	bool inside(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl);
	//  这是飞控存储格式*********************************
	typedef struct {
		double lon;	      //经度： 1e7（度），精度：小数点后 7 位，单位：度。
		double lat;       //纬度： 1e7（度），精度：小数点后 7 位，单位：度。
	}latlonDouble;

	typedef struct {
		int lon;	   //经度： 1e7（度），精度：小数点后 7 位，单位：度。
		int lat;       //纬度： 1e7（度），精度：小数点后 7 位，单位：度。
	}latlon;

	typedef struct
	{
		latlon center;
		unsigned int  radius;   //   半径（米），单位：米。
	}circle;
	typedef struct
	{
		latlon center;
		unsigned int  radius;   //   半径（米），单位：米。
		int startangle;         //   从北0度-360 顺时针
		int stopangle;
	}sector;
	typedef union
	{
			int  point[20];      // 多边形最大 10 条边
			latlon polygon[10];
			circle cir;
			sector sect;
	}Unionpoint;
	typedef struct {
		unsigned char fe;      //   总是 0xFE
		unsigned char type;    //   对应地理编码   地理编码是空域对象的唯一标识码
		unsigned char shape;   //   形状  0  :多边形    1:  圆   2 ：扇形  3 等
		unsigned char numpoint;//   多边形边数 （ 圆为 ，扇形 为1 ）
		unsigned int  cno;     //   编号   对应地理编码编号
		int minlat; //   区域最小纬度	   1e7（度）  			且存贮按此排序
		int maxlat; //   区域最小纬度	   1e7（度）
		int minlon; //   区域最小经度	   1e7（度） 			且存贮按此排序
		int maxlon; //   区域最大经度	   1e7（度）
		unsigned int minalt;      //   最小高度（米），精度：单位：米。最大高度值为 0，则表明该高度为无限高。
		unsigned int maxalt;      //   最大高度（米），精度：单位：米。最大高度值为 0，则表明该高度为无限高。
		Unionpoint pt;//   多边形边经纬度 或 圆心 半径	扇形
	}forbidden;       //   总共112个字节

	int  _maxindex;
	int  _startindex;
	int  _checkindex;
	bool _indexinit;
};

#endif /* GEOFENCE_H_ */
