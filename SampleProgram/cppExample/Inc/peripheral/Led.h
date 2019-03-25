/**
 * Led.h
 * @author yuta seya
 * @date 2019 3.25 
*/

#ifndef __LED__H
#define __LED__H

class Led{

public:
  // コンストラクタ
  Led();

  // デストラクタ
  ~Led();

  // ledを光らせる関数
  void illuminate( int led );

private:
  // led0~4をそれぞれ光らせる関数
  void set0( bool data );

  void set1( bool data );

  void set2( bool data );

  void set3( bool data );

};

#endif /* __LED__H */