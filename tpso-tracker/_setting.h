#ifndef _SETTING_H
#define _SETTING_H

#include <vector>
#include <assert.h>

class _Setting
{
public:
    _Setting()
    {
        std::vector<double> vmin(1, 0.0);
        std::vector<double> vmax(1, 1.0);
        this->init(vmin, vmax);
    }

    _Setting(std::vector<double> & minB,
             std::vector<double> & maxB)
    {
        this->init(minB, maxB);
    }

    virtual void init(std::vector<double> & minB,
                      std::vector<double> & maxB)
    {
        this->minBound = minB;
        this->maxBound = maxB;

        this->initialized = true;
    }

    virtual unsigned int getDimension() {   assert(this->initialized &&
                                                   "Please initialize Setting class!");
                                            return (unsigned int)this->minBound.size();     }

    virtual std::vector<double> & getMinBound() {   assert(this->initialized &&
                                                           "Please initialize Setting class!");
                                                    return this->minBound;      }
    virtual std::vector<double> & getMaxBound() {   assert(this->initialized &&
                                                           "Please initialize Setting class!");
                                                    return this->maxBound;      }

    virtual void setMinBound(std::vector<double> & minB)    {   this->minBound = minB;  }
    virtual void setMaxBound(std::vector<double> & maxB)    {   this->maxBound = maxB;  }

    virtual bool is_init()  {   return this->initialized;   }
protected:
    bool initialized;
    std::vector<double> minBound;
    std::vector<double> maxBound;
};

#endif // _SETTING_H
