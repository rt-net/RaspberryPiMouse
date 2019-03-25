/**
 * Switch.cpp
 * @author yuta seya
 * @date 2019 3.25 
*/

#include "Switch.h"

#include <cstdio>

/**
 * @brief スイッチクラスのコンストラクタ
 * @param なし
 * @return　なし
*/
Switch::Switch()
{
}

/**
 * @brief スイッチクラスのデストラクタ
 * @param なし
 * @return　なし
*/
Switch::~Switch()
{
}

/**
 * @brief スイッチ0の情報を取得
 * @param なし
 * @return bool スイッチの情報
*/
bool Switch::get0()
{
  bool sw_data = false;
  std::FILE *sw;
  sw = std::fopen("/dev/rtswitch0", "r");
  char data = std::fgetc(sw);
  if ( data == '0' ){
    sw_data = true;
  } else {
    sw_data = false;
  }
  std::fclose(sw);
  return sw_data;
}

/**
 * @brief スイッチ1の情報を取得
 * @param なし
 * @return bool スイッチの情報
*/
bool Switch::get1()
{
  bool sw_data = false;
  std::FILE *sw;
  sw = std::fopen("/dev/rtswitch1", "r");
  char data = std::fgetc(sw);
  if ( data == '0' ){
    sw_data = true;
  } else {
    sw_data = false;
  }
  std::fclose(sw);
  return sw_data;
}

/**
 * @brief スイッチ2の情報を取得
 * @param なし
 * @return bool スイッチの情報
*/
bool Switch::get2()
{
  bool sw_data = false;
  std::FILE *sw;
  sw = std::fopen("/dev/rtswitch2", "r");
  char data = std::fgetc(sw);
  if ( data == '0' ){
    sw_data = true;
  } else {
    sw_data = false;
  }
  std::fclose(sw);
  return sw_data;
}

