/**
 * Motor.h
 * @author yuta seya
 * @date 2019 3.25 
*/

#ifndef __MOTOR__H
#define __MOTOR__H

class Motor {
private:
  // 現在のステータス
  bool status = false;
public:
  // コンストラクタ
  Motor();

  //デストラクタ
  ~Motor();

  // ソフトウェアスイッチをオンオフする
  void setSoftwareSwitch( bool sw );

  // モーターのコントロールをする
  void control( int left, int right );

public:
  // 左モーターのコントロール
  void leftControl( int hz );

  // 右モーターのコントロール
  void rightControl( int hz );

  // ソフトウェアスイッチのオン
  void on();

  // ソフトウェアスイッチのオフ
  void off();
};

#endif /* __MOTOR__H */