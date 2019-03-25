/**
 * Motor.cpp
 * @author yuta seya
 * @date 2019 3.25 
*/

#include <cstdio>

#include "Motor.h"

#include <cstdio>

/**
 * @brief モータークラスのコンストラクタ
 * @param なし
 * @return　なし
*/
Motor::Motor()
{

}

/**
 * @brief モータークラスのデストラクタ
 * @param なし
 * @return　なし
*/
Motor::~Motor()
{
  off();
}

/**
 * @brief ソフトウェアスイッチのオンオフ
 * @param なし
 * @return なし
 * @detail 現在のソフトウェアスイッチのステータスを更新し、
 *        　ソフトウェアスイッチをステータスに合わせてオンオフを行う 
*/
void Motor::setSoftwareSwitch( bool sw )
{
  status = sw;
  if ( status ){
    on();
  } else {
    off();
  }
}

/**
 * @brief モーターのコントロールを行う
 * @param int left 左側の周波数
 * @param int right 右側の周波数
 * @return なし
*/
void Motor::control( int left, int right )
{
  if ( !status ) on();
  
  leftControl( left );
  rightControl( right );
}

/**
 * @brief ソフトウェアスイッチのオフ
 * @param なし
 * @return なし
*/
void Motor::off()
{
  std::FILE *sw;
  sw = std::fopen( "/dev/rtmotoren0","w" );
  std::fprintf( sw, "0" ); 
  std::fclose( sw );
}

/**
 * @brief ソフトウェアスイッチのオン
 * @param なし
 * @return なし
*/
void Motor::on()
{
  std::FILE *sw;
  sw = std::fopen( "/dev/rtmotoren0","w" );
  std::fprintf( sw, "1" ); 
  std::fclose( sw );
}

/**
 * @brief 左モーターのコントロールを行う
 * @param int hz 周波数
 * @return なし
*/
void Motor::leftControl( int hz )
{
  std::FILE *motor;
  motor = std::fopen( "/dev/rtmotor_raw_l0","w" );
  std::fprintf( motor, "%d", hz ); 
  std::fclose( motor );
}

/**
 * @brief 右モーターのコントロールを行う
 * @param int hz 周波数
 * @return なし
*/
void Motor::rightControl( int hz )
{
  std::FILE *motor;
  motor = std::fopen( "/dev/rtmotor_raw_r0","w" );
  std::fprintf( motor, "%d", hz ); 
  std::fclose( motor );
}