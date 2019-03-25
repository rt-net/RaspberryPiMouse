/**
 * Led.cpp
 * @author yuta seya
 * @date 2019 3.25 
*/

#include <cstdio>

#include "Led.h"

/**
 * @brief LEDクラスのコンストラクタ
 * @param なし
 * @return　なし
*/
Led::Led()
{
}

/**
 * @brief LEDクラスのデストラクタ
 * @param なし
 * @return　なし
*/
Led::~Led()
{
}

/**
 * @brief ledを点灯or消灯
 * @param int 16進数で光らせ方を指定
 * @return　なし
 * @detail led3 led2 led1 led0
 *          0    0    0    0
 *        右を下位ビットとして4bitの入力をすることで、
 *        指定のledを光らせることができる。
 *        (例) 0x0cで、led3, led2を光らせることが可能。
 *            下位0x0f以外の入力をしてもエラーは出ないが、出力は特にされない。
*/
void Led::illuminate( int led )
{
  if ( (led & 0x01) == 1 ){
    set0( true );
  } else {
    set0( false );
  }
  
  if ( ( (led & 0x02) >> 1) == 1 ){
    set1( true );
  } else {
    set1( false );
  }

  if ( ( (led & 0x04) >> 2) == 1 ){
    set2( true );
  } else {
    set2( false );
  }

   if ( ( (led & 0x08) >> 3) == 1 ){
    set3( true );
  } else {
    set3( false );
  }

}

/**
 * @brief led0を点灯or消灯
 * @param bool on or off
 * @return　なし
*/
void Led::set0( bool data )
{
  std::FILE *led;
  led = std::fopen("/dev/rtled0", "w" );

  if ( data ){
    std::fprintf( led, "1" );
  } else {
    std::fprintf( led, "0" );
  }

  fclose( led );
}

/**
 * @brief led1を点灯or消灯
 * @param bool on or off
 * @return　なし
*/
void Led::set1( bool data )
{
  std::FILE *led;
  led = std::fopen("/dev/rtled1", "w" );

  if ( data ){
    std::fprintf( led, "1" );
  } else {
    std::fprintf( led, "0" );
  }

  fclose( led );
}

/**
 * @brief led2を点灯or消灯
 * @param bool on or off
 * @return　なし
*/
void Led::set2( bool data )
{
  std::FILE *led;
  led = std::fopen("/dev/rtled2", "w" );

  if ( data ){
    std::fprintf( led, "1" );
  } else {
    std::fprintf( led, "0" );
  }

  fclose( led );
}

/**
 * @brief led3を点灯or消灯
 * @param bool on or off
 * @return　なし
*/
void Led::set3( bool data )
{
  std::FILE *led;
  led = std::fopen("/dev/rtled3", "w" );

  if ( data ){
    std::fprintf( led, "1" );
  } else {
    std::fprintf( led, "0" );
  }

  fclose( led );
}