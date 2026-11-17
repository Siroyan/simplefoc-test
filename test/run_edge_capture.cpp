#include "RunEdgeCapture.h"
#include <cstdlib>
#include <iostream>

struct Edge {uint32_t us;int32_t count;uint8_t state;};
void require(bool ok,const char* why) {if(!ok){std::cerr<<"FAIL: "<<why<<'\n';std::exit(1);}}
int main() {
  volatile Edge storage[4]{};
  RunEdgeCapture<Edge,4> capture(storage);Edge row{};
  require(!capture.busy() && !capture.read(0,row),"inactive has no readable capture");
  require(!capture.begin(0,4),"invalid initial AB state rejected");
  require(capture.begin(-20,2),"borrow inactive storage");
  require(!capture.begin(0,0) && !capture.release(),"active capture cannot be reset/released");
  for(int n=1;n<=7;++n)capture.edge(n*100,-20+n,n%4);
  require(!capture.read(0,row),"do not read a changing ISR ring");
  capture.freeze(750,-13);
  require(capture.pending && !capture.active && capture.total==7 && capture.size==4,"freeze retains bounded tail and full count");
  for(int n=0;n<4;++n) {
    require(capture.read(static_cast<uint16_t>(n),row),"read chronological frozen edge");
    require(row.us==uint32_t((n+4)*100) && row.count==-16+n,"wrapped tail is chronological and signed");
  }
  capture.edge(800,100,0);capture.freeze(900,100);
  require(capture.endUs==750 && capture.endCount==-13 && capture.total==7,"frozen evidence cannot be changed by a late ISR or repeated stop");
  require(!capture.begin(1,1),"pending output prevents storage reuse");
  require(capture.release() && !capture.busy(),"only completed transmission releases manual storage");
  require(capture.begin(80,1),"a later owner can acquire released storage");
  capture.freeze(10,80);
  require(capture.size==0 && capture.total==0 && !capture.read(0,row),"empty capture remains explicit");
  std::cout<<"PASS: borrowed storage, ring wrap, signed counts, immutable stop snapshot, no active/pending reuse, empty capture\n";
}
