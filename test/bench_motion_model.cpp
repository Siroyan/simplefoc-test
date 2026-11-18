// Candidate comparison, not an identified plant or hardware safety proof.
#include <BenchMotion.h>
#include <EdgeVelocity.h>
#include <iostream>
#include <cmath>
int main(){
  constexpr double dt=.0001,tick=6.283185307179586/4000;
  std::cout<<"P,K_rad_s2_per_nominal_A,current_tau_s,cog_nominal_A,phase,complete,rms_error_joint_deg,peak_motor_rpm,switches\n";
  for(double gain:{1000.,3000.,6000.})for(double tau:{.001,.002,.003})for(double cog:{0.,.1,.2})for(double phase:{0.,1.57079632679}){
    BenchMotion control;control.reset(0,1);EdgeVelocity sensor;sensor.reset(0);
    double angle=0,velocity=0,iq=0,measured=0,peak=0,sum=0;uint32_t samples=0,switches=0;int32_t count=0;float previousTarget=0;
    bool done=true;
    for(uint32_t us=100;us<=BenchMotion::durationUs;us+=100){
      iq+=(control.currentTarget-iq)*dt/(tau+dt);
      double torque=iq-cog*std::sin(14*angle+phase);
      if(std::abs(velocity)<.01 && std::abs(torque)<.025)velocity=0;
      else velocity+=(gain*(torque-.025*(velocity>0 ? 1:-1))-8*velocity)*dt;
      const double before=angle;angle+=velocity*dt;
      const int32_t next=static_cast<int32_t>(std::floor(angle/tick));
      while(count!=next){
        const bool forward=next>count;const double crossing=(forward ? count+1:count)*tick;
        const double fraction=(crossing-before)/(angle-before);
        count+=forward ? 1:-1;
        sensor.edge(static_cast<uint32_t>(count),us-100+static_cast<uint32_t>(std::max(0.,std::min(1.,fraction))*100));
      }
      if(us%500==0){const float raw=sensor.update(sensor.snapshot(),us);measured+=(raw-measured)*.0005/.0015;}
      const float request=control.update(us,us,static_cast<float>(count*tick/8),static_cast<float>(measured/8),static_cast<float>(velocity));
      if(request*previousTarget<0)++switches;previousTarget=request;
      if(control.invalid){done=false;break;}
      if(control.phase==1 || control.phase==3){const double e=(control.targetJoint-count*tick/8)/BenchMotion::degree;sum+=e*e;peak=std::max(peak,std::abs(velocity));++samples;}
    }
    std::cout<<BenchMotion::velocityP<<','<<gain<<','<<tau<<','<<cog<<','<<phase<<','<<done<<','<<(samples ? std::sqrt(sum/samples):0)<<','<<peak*30/3.141592653589793<<','<<switches<<'\n';
  }
}
