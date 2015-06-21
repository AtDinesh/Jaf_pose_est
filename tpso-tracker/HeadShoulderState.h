#ifndef HEADSHOULDERSTATE_H
#define HEADSHOULDERSTATE_H

#include <_state.h>

#include <cmath>

/*
 *%normalisation du sinus et cosinus

X=Xold;

%head pan
X(4,:) = Xold(4,:)./sqrt(Xold(4,:).^2+Xold(5,:).^2);
X(5,:) = Xold(5,:)./sqrt(Xold(4,:).^2+Xold(5,:).^2);

X(4,:);
X(5,:);

%waitforbuttonpress

%head tilt
X(6,:) = Xold(6,:)./sqrt(Xold(6,:).^2+Xold(7,:).^2);
X(7,:) = Xold(7,:)./sqrt(Xold(6,:).^2+Xold(7,:).^2);

%head roll
X(8,:) = Xold(8,:)./sqrt(Xold(8,:).^2+Xold(9,:).^2);
X(9,:) = Xold(9,:)./sqrt(Xold(8,:).^2+Xold(9,:).^2);

%shoulders pan
X(10,:) = Xold(10,:)./sqrt(Xold(10,:).^2+Xold(11,:).^2);
X(11,:) = Xold(11,:)./sqrt(Xold(10,:).^2+Xold(11,:).^2);
*/

//template <>
class HeadShoulderState : public _State<0, 11>
{

public:

    HeadShoulderState():_State<0,11>() { }
    HeadShoulderState(const _State<0, 11> & state):_State<0,11>(state)    { }
    HeadShoulderState(_State<0, 11> & state):_State<0,11>(state)          { }
    HeadShoulderState(std::vector<int> &discrParams,
                      std::vector<double> &contParams)
        :_State<0,11>(discrParams, contParams)                           { }


    inline virtual
    void limits(const std::vector<double> & min,
                const std::vector<double> & max)    //dummy inputs for the sake of
                                                    //keeping the interface
    {

        std::vector<double> oldVal = this->continousParams_;
        //pos - 0,1,2
        //angles- 3 to 10

        double sqrt34   = sqrt(oldVal[3]*oldVal[3] + oldVal[4]*oldVal[4]);
        double sqrt56   = sqrt(oldVal[5]*oldVal[5] + oldVal[6]*oldVal[6]);
        double sqrt78   = sqrt(oldVal[7]*oldVal[7] + oldVal[8]*oldVal[8]);
        double sqrt910  = sqrt(oldVal[9]*oldVal[9] + oldVal[10]*oldVal[10]);

        this->continousParams_[3] = oldVal[3]/sqrt34;
        this->continousParams_[4] = oldVal[4]/sqrt34;
        this->continousParams_[5] = oldVal[5]/sqrt56;
        this->continousParams_[6] = oldVal[6]/sqrt56;
        this->continousParams_[7] = oldVal[7]/sqrt78;
        this->continousParams_[8] = oldVal[8]/sqrt78;
        this->continousParams_[9] = oldVal[9]/sqrt910;
        this->continousParams_[10] = oldVal[10]/sqrt910;
    }

};

#endif // HEADSHOULDERSTATE_H
