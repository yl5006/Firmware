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

#pragma once

#include <float.h>

#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <px4_defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_air_data.h>

#define GEOFENCE_FILENAME PX4_STORAGEDIR"/etc/geofence.txt"

#if defined(__PX4_NUTTX)
#define GEOFENCE_EWT PX4_ROOTFSDIR"/etc/extras/geo_nofly.data"
#elif defined(__PX4_POSIX)
#define GEOFENCE_EWT PX4_ROOTFSDIR"/eeprom/geo_nofly.data"
#endif
class Navigator;

class Geofence : public ModuleParams
{
public:
	Geofence(Navigator *navigator);
	Geofence(const Geofence &) = delete;
	Geofence &operator=(const Geofence &) = delete;
	~Geofence();

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
	 * update the geofence from dataman.
	 * It's generally not necessary to call this as it will automatically update when the data is changed.
	 */
	void updateFence();

	/**
	 * Return whether the system obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const vehicle_global_position_s &global_position,
		   const vehicle_gps_position_s &gps_position, const home_position_s home_pos, bool home_position_set);

	/**
	 * Return whether a mission item obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const struct mission_item_s &mission_item);

	int clearDm();

	bool valid();

	/**
	 * Load a single inclusion polygon, replacing any already existing polygons.
	 * The file has one of the following formats:
	 * - Decimal Degrees:
	 * 0 900
	 * 47.475273548913222 8.52672100067138672
	 * 47.4608261578541359 8.53414535522460938
	 * 47.4613484218861217 8.56444358825683594
	 * 47.4830758091035534 8.53470325469970703
	 *
	 * - Degree-Minute-Second:
	 * 0 900
	 * DMS -26 -34 -10.4304 151 50 14.5428
	 * DMS -26 -34 -11.8416 151 50 21.8580
	 * DMS -26 -34 -36.5628 151 50 28.1112
	 * DMS -26 -34 -37.1640 151 50 24.1620
	 *
	 * Where the first line is min, max altitude in meters AMSL.
	 */
	int loadFromFile(const char *filename);
	int loadFromEwtFile(const struct vehicle_global_position_s &global_position);
	bool intsideEwtFile(const struct vehicle_global_position_s &global_position);

	bool isEmpty() { return _num_polygons == 0; }

	int getAltitudeMode() { return _param_altitude_mode.get(); }
	int getSource() { return _param_source.get(); }
	int getGeofenceAction() { return _param_action.get(); }

	bool isHomeRequired();

	/**
	 * print Geofence status to the console
	 */
	void printStatus();

private:
	Navigator	*_navigator{nullptr};

	hrt_abstime _last_horizontal_range_warning{0};
	hrt_abstime _last_vertical_range_warning{0};

	float _altitude_min{0.0f};
	float _altitude_max{0.0f};

	struct PolygonInfo {
		uint16_t fence_type; ///< one of MAV_CMD_NAV_FENCE_* (can also be a circular region)
		uint16_t dataman_index;
		union {
			uint16_t vertex_count;
			float circle_radius;
		};
	};
	PolygonInfo *_polygons{nullptr};
	int _num_polygons{0};

	map_projection_reference_s _projection_reference = {}; ///< reference to convert (lon, lat) to local [m]

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::GF_ACTION>) _param_action,
		(ParamInt<px4::params::GF_ALTMODE>) _param_altitude_mode,
		(ParamInt<px4::params::GF_SOURCE>) _param_source,
		(ParamInt<px4::params::GF_COUNT>) _param_counter_threshold,
		(ParamFloat<px4::params::GF_MAX_HOR_DIST>) _param_max_hor_distance,
		(ParamFloat<px4::params::GF_MAX_VER_DIST>) _param_max_ver_distance
	)

	uORB::Subscription<vehicle_air_data_s>	_sub_airdata;

	int _outside_counter{0};
	uint16_t _update_counter{0}; ///< dataman update counter: if it does not match, we polygon data was updated

	/**
	 * implementation of updateFence(), but without locking
	 */
	void _updateFence();

	/**
	 * Check if a point passes the Geofence test.
	 * This takes all polygons and minimum & maximum altitude into account
	 *
	 * The check passes if: (inside(polygon_inclusion_1) || inside(polygon_inclusion_2) || ... ) &&
	 *                       !inside(polygon_exclusion_1) && !inside(polygon_exclusion_2) && ...
	 *                       && (altitude within [min, max])
	 *                  or: no polygon configured
	 * @return result of the check above (false for a geofence violation)
	 */
	bool checkPolygons(double lat, double lon, float altitude);

	/**
	 * Check if a point passes the Geofence test.
	 * In addition to checkPolygons(), this takes all additional parameters into account.
	 *
	 * @return false for a geofence violation
	 */
	bool checkAll(double lat, double lon, float altitude);

	bool checkAll(const vehicle_global_position_s &global_position);
	bool checkAll(const vehicle_global_position_s &global_position, float baro_altitude_amsl);

	/**
	 * Check if a single point is within a polygon
	 * @return true if within polygon
	 */
	bool insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude);

	/**
	 * Check if a single point is within a circle
	 * @param polygon must be a circle!
	 * @return true if within polygon the circle
	 */
	bool insideCircle(const PolygonInfo &polygon, double lat, double lon, float altitude);
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
		int  radius;   //   半径（米），单位：米。
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
