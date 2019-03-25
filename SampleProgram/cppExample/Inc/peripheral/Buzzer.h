/**
 * Buzzer.h
 * @author yuta seya
 * @date 2019 3.25 
*/

#ifndef __BUZZER__H
#define __BUZZER__H

class Buzzer{
private:

public:
  // 音の周波数の列挙
  
  #define C "261"
  #define C_Sharp "277"
  #define D "294"
  #define D_Sharp "311"
  #define E "330"
  #define F "349"
  #define F_Sharp "370"
  #define G "392"
  #define G_Sharp "415"
  #define A "440"
  #define A_Sharp "466"
  #define B "494"
  
  // コンストラクタ
  Buzzer();

  // デストラクタ
  ~Buzzer();

  void on( const char *scale, int wait_time );

private:
  void off();

};

#endif /* __BUZZER__H */