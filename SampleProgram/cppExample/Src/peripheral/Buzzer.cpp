/**
 * Buzzer.cpp
 * @author yuta seya
 * @date 2019 3.25 
*/

#include "Buzzer.h"

#include <cstdio>
#include <unistd.h>

/**
 * @brief ブザークラスのコンストラクタ
 * @param なし
 * @return　なし
*/
Buzzer::Buzzer()
{

}

/**
 * @brief ブザークラスのデストラクタ
 * @param なし
 * @return　なし
*/
Buzzer::~Buzzer()
{

}

/**
 * @brief ブザーの音を出力する
 * @param char スケール(defineされているものを使用)
 * @param int 鳴らす時間(ms)
 * @return　なし
 * @detail 指定されたスケールで、指定された時間ブザーの音を鳴らす。
*/
void Buzzer::on( const char *scale, int wait_time )
{
  std::FILE *bz;
  bz = std::fopen( "/dev/rtbuzzer0", "w" );
  std::fprintf( bz, scale );
  std::fclose( bz );
  usleep( wait_time * 1000 );
  off();
}

/**
 * @brief ブザーの音を消す
 * @param なし
 * @return　なし
*/
void Buzzer::off()
{
  std::FILE *bz;
  bz = std::fopen( "/dev/rtbuzzer0", "w" );
  std::fprintf( bz, "0" );
  std::fclose( bz );
}