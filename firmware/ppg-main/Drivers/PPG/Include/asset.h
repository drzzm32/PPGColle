#ifndef __ASSET_H_
#define __ASSET_H_


#define PPG_TOP_WIDTH		20
#define PPG_TOP_HEIGHT		32

#define PPG_TOP_BT			0
#define PPG_TOP_LED_ON		1
#define PPG_TOP_USB			2
#define PPG_TOP_BAT			3
#define PPG_TOP_BT_OK		4
#define PPG_TOP_LED			5
#define PPG_TOP_BAT_LOW		6
#define PPG_TOP_BAT_CHG		7

#define PPG_WEATHER_WIDTH	48
#define PPG_WEATHER_HEIGHT	48

#define PPG_WEATHER_SUNNY	0
#define PPG_WEATHER_CLOUDY	1
#define PPG_WEATHER_FOG		2
#define PPG_WEATHER_PCLOUDY	3
#define PPG_WEATHER_RAINY	4
#define PPG_WEATHER_WINDY	5

const unsigned char* getBlank();
const unsigned char* getBackground();
const unsigned char* getTopIcon(unsigned char index);
const unsigned char* getWeatherIcon(unsigned char index);


#endif
