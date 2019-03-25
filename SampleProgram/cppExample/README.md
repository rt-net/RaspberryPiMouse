#C++の周辺機能を使用するサンプルプログラム

##ディレクトリ構造
```
- cppExample
  - Inc
    - peripheral
      - *.h
  - Src
    - peripheral
      - *.cpp
  - .gitignore
  - main.cpp
  - makefile
  - README.md
```
##使用方法
makefileがおかれているディレクトリまで移動し、makeコマンドをたたくことでコンパイルが開始し、ディレクトリ名の実行ファイルが生成されます。
```
>　$make
```
初期の状態では、cppExampleという実行ファイルが生成され、
```
> $make run
```
とすることで実行が可能です。
ディレクトリ名を変えた場合は、makefile内の
```
> run:
>	  build/cppExample
```
のcppExampleのところを変更したディレクトリ名にしてください。
初期の状態では、スイッチをおすとブザがなり、ledが一個ずつ光るというプログラムが入っています。
関数の引数や動作は、ソースコードを読んでください。

##参考文献
[makefile] (https://gist.github.com/urin/5971408#file-makefile)
