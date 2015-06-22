// - Classes for 1D and 2D convolution stencils
// - Pre-defined convolution stencils for binomial filters
// - Pre-defined convolution stencils for 1st, 2nd, 3rd and 4th derivatives up to order 10
// - Functions for convolution
//
// Author: Thomas Brox

#ifndef CFILTER
#define CFILTER

#include <math.h>
#include "Vector.h"
#include "Matrix.h"

// CFilter is an extention of Vector. It has an additional property Delta
// which shifts the data to the left (a vector always begins with index 0).
// This enables a filter's range to go from A to B where A can also
// be less than zero.
//
// Example:
// CFilter<double> filter(3,1);
// filter = 1.0;
// cout << filter(-1) << ", " << filter(0) << ", " << filter(1) << endl;
//
// CFilter2D behaves the same way as CFilter but is an extension of Matrix

template <class T>
class CFilter : public Vector<T> {
public:
    // constructor
    inline CFilter(const int aSize, const int aDelta = 0);
    // copy constructor
    CFilter(const CFilter<T>& aCopyFrom);
    // constructor initialized by a vector
    CFilter(const Vector<T>& aCopyFrom, const int aDelta = 0);

    // Access to the filter's values
    inline T& operator()(const int aIndex) const;
    inline T& operator[](const int aIndex) const;
    // Copies a filter into this filter
    CFilter<T>& operator=(const CFilter<T>& aCopyFrom);

    // Access to the filter's delta
    inline int delta() const;
    // Access to the filter's range A<=i<B
    inline int A() const;
    inline int B() const;
    // Returns the sum of all filter co-efficients (absolutes)
    T sum() const;
    // Shifts the filter
    inline void shift(int aDelta);
protected:
    int mDelta;
};

template <class T>
class CFilter2D : public Matrix<T> {
public:
    // constructor
    inline CFilter2D();
    inline CFilter2D(const int aXSize, const int aYSize, const int aXDelta = 0, const int aYDelta = 0);
    // copy contructor
    CFilter2D(const CFilter2D<T>& aCopyFrom);
    // constructor initialized by a matrix
    CFilter2D(const Matrix<T>& aCopyFrom, const int aXDelta = 0, const int aYDelta = 0);
    // Normalize sum of values to 1.0
    void normalizeSum();
    // Moves the filter's center
    void shift(int aXDelta, int aYDelta);

    // Access to filter's values
    inline T& operator()(const int ax, const int ay) const;
    // Copies a filter into this filter
    CFilter2D<T>& operator=(const CFilter2D<T>& aCopyFrom);

    // Access to the filter's delta
    inline int deltaX() const;
    inline int deltaY() const;
    // Access to the filter's range A<=i<B
    inline int AX() const;
    inline int BX() const;
    inline int AY() const;
    inline int BY() const;
    // Returns the sum of all filter co-efficients (absolutes)
    T sum() const;
protected:
    int mDeltaX;
    int mDeltaY;
};

namespace NFilter {

// Linear 1D filtering

// Convolution of the vector aVector with aFilter
// The result will be written into aVector, so its initial values will get lost
template <class T> inline void filter(Vector<T>& aVector, const CFilter<T>& aFilter);
// Convolution of the vector aVector with aFilter, the initial values of aVector will persist.
template <class T> void filter(const Vector<T>& aVector, Vector<T>& aResult, const CFilter<T>& aFilter);

// Linear 2D filtering

// Convolution of the matrix aMatrix with aFilter, aFilter should be a separable filter
// The result will be written into aMatrix, so its initial values will get lost
template <class T> inline void filter(Matrix<T>& aMatrix, const CFilter<T>& aFilterX, const CFilter<T>& aFilterY);
// Convolution of the matrix aMatrix with aFilter, aFilter must be separable
// The initial values of aMatrix will persist.
template <class T> inline void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter<T>& aFilterX, const CFilter<T>& aFilterY);

// Convolution of the matrix aMatrix with aFilter only in x-direction, aDummy can be set to 1
// The result will be written into aMatrix, so its initial values will get lost
template <class T> inline void filter(Matrix<T>& aMatrix, const CFilter<T>& aFilter, const int aDummy);
// Convolution of the matrix aMatrix with aFilter only in x-direction, aDummy can be set to 1
// The initial values of aMatrix will persist.
template <class T> void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter<T>& aFilter, const int aDummy);
// Convolution of the matrix aMatrix with aFilter only in y-direction, aDummy can be set to 1
// The result will be written into aMatrix, so its initial values will get lost
template <class T> inline void filter(Matrix<T>& aMatrix, const int aDummy, const CFilter<T>& aFilter);
// Convolution of the matrix aMatrix with aFilter only in y-direction, aDummy can be set to 1
// The initial values of aMatrix will persist.
template <class T> void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const int aDummy, const CFilter<T>& aFilter);

// Convolution of the matrix aMatrix with aFilter
// The result will be written to aMatrix, so its initial values will get lost
template <class T> inline void filter(Matrix<T>& aMatrix, const CFilter2D<T>& aFilter);
// Convolution of the matrix aMatrix with aFilter, the initial values of aMatrix will persist
template <class T> void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter2D<T>& aFilter);

// Convolution with a rectangle -> approximation of Gaussian
template <class T> inline void boxFilterX(Matrix<T>& aMatrix, int aWidth);
template <class T> void boxFilterX(const Matrix<T>& aMatrix, Matrix<T>& aResult, int aWidth);
template <class T> inline void boxFilterY(Matrix<T>& aMatrix, int aWidth);
template <class T> void boxFilterY(const Matrix<T>& aMatrix, Matrix<T>& aResult, int aWidth);

}

// Common filters

template <class T>
class CGauss : public CFilter<T> {
public:
    CGauss(const int aSize, const int aDegreeOfDerivative);
};

template <class T>
class CSmooth : public CFilter<T> {
public:
    CSmooth(float aSigma, float aPrecision);
};

template <class T>
class CDerivative : public CFilter<T> {
public:
    CDerivative(const int aSize);
};

template <class T>
class CHighOrderDerivative : public CFilter<T> {
public:
    CHighOrderDerivative(int aOrder, int aSize);
};

template <class T>
class CGaborReal : public CFilter2D<T> {
public:
    CGaborReal(float aFrequency, float aAngle, float aSigma1 = 3.0, float aSigma2 = 3.0);
};

template <class T>
class CGaborImaginary : public CFilter2D<T> {
public:
    CGaborImaginary(float aFrequency, float aAngle, float aSigma1 = 3.0, float aSigma2 = 3.0);
};


// I M P L E M E N T A T I O N ------------------------------------------------
//
// You might wonder why there is implementation code in a header file.
// The reason is that not all C++ compilers yet manage separate compilation
// of templates. Inline functions cannot be compiled separately anyway.
// So in this case the whole implementation code is added to the header
// file.
// Users should ignore everything that's beyond this line :)
// ----------------------------------------------------------------------------

// C F I L T E R --------------------------------------------------------------
// P U B L I C ----------------------------------------------------------------
// constructor
template <class T>
inline CFilter<T>::CFilter(const int aSize, const int aDelta)
    : Vector<T>(aSize),mDelta(aDelta) {
}

// copy constructor
template <class T>
CFilter<T>::CFilter(const CFilter<T>& aCopyFrom)
    : Vector<T>(aCopyFrom.mSize),mDelta(aCopyFrom.mDelta) {
    for (register int i = 0; i < this->mSize; i++)
        this->data()[i] = aCopyFrom.data()[i];
}

// constructor initialized by a vector
template <class T>
CFilter<T>::CFilter(const Vector<T>& aCopyFrom, const int aDelta)
    : Vector<T>(aCopyFrom.size()),mDelta(aDelta) {
    for (register int i = 0; i < this->mSize; i++)
        this->data()[i] = aCopyFrom(i);
}

// operator()
template <class T>
inline T& CFilter<T>::operator()(const int aIndex) const {
#ifdef DEBUG
    assert (aIndex >= A() || aIndex < B())
#endif
    return this->data()[aIndex+mDelta];
}

// operator[]
template <class T>
inline T& CFilter<T>::operator[](const int aIndex) const {
    return operator()(aIndex);
}

// operator=
template <class T>
CFilter<T>& CFilter<T>::operator=(const CFilter<T>& aCopyFrom) {
    if (this != &aCopyFrom) {
        delete[] this->data();
        this->mSize = aCopyFrom.mSize;
        mDelta = aCopyFrom.mDelta;
        this->data() = new T[this->mSize];
        for (register int i = 0; i < this->mSize; i++)
            this->data()[i] = aCopyFrom.data()[i];
    }
    return *this;
}

// delta
template <class T>
inline int CFilter<T>::delta() const {
    return mDelta;
}

// A
template <class T>
inline int CFilter<T>::A() const {
    return -mDelta;
}

// B
template <class T>
inline int CFilter<T>::B() const {
    return this->mSize-mDelta;
}

// sum
template <class T>
T CFilter<T>::sum() const {
    T aResult = 0;
    for (int i = 0; i < this->mSize; i++)
        aResult += fabs(this->data()[i]);
    return aResult;
}

// shift
template <class T>
inline void CFilter<T>::shift(int aDelta) {
    mDelta += aDelta;
}

// C F I L T E R 2 D -----------------------------------------------------------
// P U B L I C ----------------------------------------------------------------
// constructor
template <class T>
inline CFilter2D<T>::CFilter2D()
    : Matrix<T>(),mDeltaX(0),mDeltaY(0) {
}

template <class T>
inline CFilter2D<T>::CFilter2D(const int aXSize, const int aYSize, const int aDeltaX, const int aDeltaY)
    : Matrix<T>(aXSize,aYSize),mDeltaX(aDeltaX),mDeltaY(aDeltaY) {
}

// copy constructor
template <class T>
CFilter2D<T>::CFilter2D(const CFilter2D<T>& aCopyFrom)
    : Matrix<T>(aCopyFrom.xSize(),aCopyFrom.mYSize),mDeltaX(aCopyFrom.mDeltaX,aCopyFrom.mDeltaY) {
    for (int i = 0; i < this->xSize()*this->mYSize; i++)
        this->data()[i] = aCopyFrom.data()[i];
}

// constructor initialized by a matrix
template <class T>
CFilter2D<T>::CFilter2D(const Matrix<T>& aCopyFrom, const int aDeltaX, const int aDeltaY)
    : Matrix<T>(aCopyFrom.xSize(),aCopyFrom.ySize()),mDeltaX(aDeltaX),mDeltaY(aDeltaY) {
    for (register int i = 0; i < this->xSize()*this->mYSize; i++)
        this->data()[i] = aCopyFrom.data()[i];
}

// normalizeSum
template <class T>
void CFilter2D<T>::normalizeSum() {
    int aSize = this->size();
    T aSum = 0;
    for (int i = 0; i < aSize; i++)
        aSum += this->data()[i];
    T invSum = 1.0/aSum;
    for (int i = 0; i < aSize; i++)
        this->data()[i] *= invSum;
}

// shift
template <class T>
void CFilter2D<T>::shift(int aXDelta, int aYDelta) {
    mDeltaX = aXDelta;
    mDeltaY = aYDelta;
}

// operator()
template <class T>
inline T& CFilter2D<T>::operator()(const int ax, const int ay) const {
#ifdef DEBUG
    assert (ax >= AX() || ax < BX() || ay >= AY() || ay < BY)
#endif
    return this->data()[(ay+mDeltaY)*this->xSize()+ax+mDeltaX];
}

// operator=
template <class T>
CFilter2D<T>& CFilter2D<T>::operator=(const CFilter2D<T>& aCopyFrom) {
    if (this != &aCopyFrom) {
        delete[] this->data();
        this->xSize() = aCopyFrom.xSize();
        this->mYSize = aCopyFrom.mYSize;
        mDeltaX = aCopyFrom.mDeltaX;
        mDeltaY = aCopyFrom.mDeltaY;
        this->data() = new T[this->xSize()*this->mYSize];
        for (register int i = 0; i < this->xSize()*this->mYSize; i++)
            this->data()[i] = aCopyFrom.data()[i];
    }
    return *this;
}

// deltaX
template <class T>
inline int CFilter2D<T>::deltaX() const {
    return mDeltaX;
}

// deltaY
template <class T>
inline int CFilter2D<T>::deltaY() const {
    return mDeltaY;
}

// AX
template <class T>
inline int CFilter2D<T>::AX() const {
    return -mDeltaX;
}

// AY
template <class T>
inline int CFilter2D<T>::AY() const {
    return -mDeltaY;
}

// BX
template <class T>
inline int CFilter2D<T>::BX() const {
    return this->xSize()-mDeltaX;
}

// BY
template <class T>
inline int CFilter2D<T>::BY() const {
    return this->ySize()-mDeltaY;
}

// sum
template <class T>
T CFilter2D<T>::sum() const {
    T aResult = 0;
    for (int i = 0; i < this->xSize()*this->mYSize; i++)
        aResult += abs(this->data()[i]);
    return aResult;
}

// C G A U S S -----------------------------------------------------------------
template <class T>
CGauss<T>::CGauss(const int aSize, const int aDegreeOfDerivative)
    : CFilter<T>(aSize,aSize >> 1) {
                                Vector<int> *oldData;
Vector<int> *newData;
Vector<int> *temp;
oldData = new Vector<int>(aSize);
newData = new Vector<int>(aSize);

(*oldData)(0) = 1;
(*oldData)(1) = 1;

for (int i = 2; i < aSize-aDegreeOfDerivative; i++) {
    (*newData)(0) = 1;
    for (int j = 1; j < i; j++)
        (*newData)(j) = (*oldData)(j)+(*oldData)(j-1);
    (*newData)(i) = 1;
    temp = oldData;
    oldData = newData;
    newData = temp;
}
for (int i = aSize-aDegreeOfDerivative; i < aSize; i++) {
    (*newData)(0) = 1;
    for (int j = 1; j < i; j++)
        (*newData)(j) = (*oldData)(j)-(*oldData)(j-1);
    (*newData)(i) = -(*oldData)(i-1);
    temp = oldData;
    oldData = newData;
    newData = temp;
}

int aSum = 0;
for (int i = 0; i < aSize; i++)
aSum += abs((*oldData)(i));
double aInvSum = 1.0/aSum;
for (int i = 0; i < aSize; i++)
this->data()[aSize-1-i] = (*oldData)(i)*aInvSum;

delete newData;
delete oldData;
}

// C S M O O T H ---------------------------------------------------------------
template <class T>
CSmooth<T>::CSmooth(float aSigma, float aPrecision)
    : CFilter<T>(2*(int)ceil(aPrecision*aSigma)+1,(int)ceil(aPrecision*aSigma)) {
    for (int i = 0; i <= (this->mSize >> 1); i++) {
        T aTemp = exp(i*i/(-2.0*aSigma*aSigma))/(aSigma*sqrt(2.0*M_PI));
        this->operator()(i) = aTemp;
        this->operator()(-i) = aTemp;
    }
    T invSum = 1.0/this->sum();
    for (int i = 0; i < this->mSize; i++)
        this->data()[i] *= invSum;
}

// C D E R I V A T I V E -------------------------------------------------------
template <class T>
CDerivative<T>::CDerivative(const int aSize)
    : CFilter<T>(aSize,(aSize-1) >> 1) {
                                    switch (aSize) {
                                    case 2:
                                    this->data()[0] = -1;
this->data()[1] =  1;
break;
case 3:
this->data()[0] = -0.5;
this->data()[1] = 0;
this->data()[2] = 0.5;
break;
case 4:
this->data()[0] =   0.041666666666666666666666666666667;
this->data()[1] =  -1.125;
this->data()[2] =   1.125;
this->data()[3] =  -0.041666666666666666666666666666667;
break;
case 5:
this->data()[0] =  0.083333333333;
this->data()[1] = -0.66666666666;
this->data()[2] =  0;
this->data()[3] =  0.66666666666;
this->data()[4] = -0.083333333333;
break;
case 6:
this->data()[0] = -0.0046875;
this->data()[1] =  0.0651041666666666666666666666666667;
this->data()[2] = -1.171875;
this->data()[3] =  1.171875;
this->data()[4] = -0.0651041666666666666666666666666667;
this->data()[5] =  0.0046875;
break;
case 7:
this->data()[0] = -0.016666666666666666666666666666667;
this->data()[1] =  0.15;
this->data()[2] = -0.75;
this->data()[3] =  0;
this->data()[4] =  0.75;
this->data()[5] = -0.15;
this->data()[6] =  0.016666666666666666666666666666667;
break;
case 8:
this->data()[0] =  6.9754464285714285714285714285714e-4;
this->data()[1] = -0.0095703125;
this->data()[2] =  0.079752604166666666666666666666667;
this->data()[3] = -1.1962890625;
this->data()[4] =  1.1962890625;
this->data()[5] = -0.079752604166666666666666666666667;
this->data()[6] =  0.0095703125;
this->data()[7] = -6.9754464285714285714285714285714e-4;
break;
case 9:
this->data()[0] =  0.0035714285714285714285714285714286;
this->data()[1] = -0.038095238095238095238095238095238;
this->data()[2] =  0.2;
this->data()[3] = -0.8;
this->data()[4] =  0;
this->data()[5] =  0.8;
this->data()[6] = -0.2;
this->data()[7] =  0.038095238095238095238095238095238;

this->data()[8] = -0.0035714285714285714285714285714286;
break;
case 10:
this->data()[0] = -1.1867947048611111111111111111111e-4;
this->data()[1] =  0.0017656598772321428571428571428571;
this->data()[2] = -0.0138427734375;
this->data()[3] =  0.0897216796875;
this->data()[4] = -1.21124267578125;
this->data()[5] =  1.21124267578125;
this->data()[6] = -0.0897216796875;
this->data()[7] =  0.0138427734375;
this->data()[8] = -0.0017656598772321428571428571428571;
this->data()[9] =  1.1867947048611111111111111111111e-4;
break;
}
}

// C H I G H O R D E R D E R I V A T I V E -------------------------------------
template <class T>
CHighOrderDerivative<T>::CHighOrderDerivative(int aOrder, int aSize)
    : CFilter<T>(aSize,(aSize-1) >> 1) {
                                    switch (aSize) {
                                    case 3:
                                    switch (aOrder) {
                                    case 2:
                                    this->data()[0] =  1;
this->data()[1] =  -2;
this->data()[2] =  1;
break;

}
break;
case 4:
switch (aOrder) {
    case 2:
        this->data()[0] =   0.25;
        this->data()[1] =  -0.25;
        this->data()[2] =  -0.25;
        this->data()[3] =   0.25;
        break;
    case 3:
        this->data()[0] =  -0.25;
        this->data()[1] =   0.75;
        this->data()[2] =  -0.75;
        this->data()[3] =   0.25;
        break;


}
break;
case 5:
switch (aOrder) {
    case 2:
        this->data()[0] = -0.083333333333333333333333333333333;
        this->data()[1] =  1.3333333333333333333333333333333;
        this->data()[2] = -2.5;
        this->data()[3] =  1.3333333333333333333333333333333;
        this->data()[4] = -0.083333333333333333333333333333333;
        break;
    case 3:
        this->data()[0] = -0.5;
        this->data()[1] =  1;
        this->data()[2] =  0;
        this->data()[3] = -1;
        this->data()[4] =  0.5;
        break;
    case 4:
        this->data()[0] =  1;
        this->data()[1] = -4;
        this->data()[2] =  6;
        this->data()[3] = -4;
        this->data()[4] =  1;
        break;


}
break;
case 6:
switch (aOrder) {
    case 2:
        this->data()[0] = -0.052083333333333333333333333333333;
        this->data()[1] =  0.40625;
        this->data()[2] = -0.35416666666666666666666666666667;
        this->data()[3] = -0.35416666666666666666666666666667;
        this->data()[4] =  0.40625;
        this->data()[5] = -0.052083333333333333333333333333333;
        break;
    case 3:
        this->data()[0] =  0.03125;
        this->data()[1] = -0.40625;
        this->data()[2] =  1.0625;
        this->data()[3] = -1.0625;
        this->data()[4] =  0.40625;
        this->data()[5] = -0.03125;
        break;
    case 4:
        this->data()[0] =  0.0625;
        this->data()[1] = -0.1875;
        this->data()[2] =  0.125;
        this->data()[3] =  0.125;
        this->data()[4] = -0.1875;
        this->data()[5] =  0.0625;
        break;


}
break;
case 7:
switch (aOrder) {
    case 2:
        this->data()[0] =  0.011111111111111111111111111111111;
        this->data()[1] = -0.15;
        this->data()[2] =  1.5;
        this->data()[3] = -2.6666666666666666666666666666667;
        this->data()[4] =  1.5;
        this->data()[5] = -0.15;
        this->data()[6] =  0.011111111111111111111111111111111;
        break;
    case 3:
        this->data()[0] =  0.125;
        this->data()[1] = -1;
        this->data()[2] =  1.625;
        this->data()[3] =  0;
        this->data()[4] = -1.625;
        this->data()[5] =  1;
        this->data()[6] = -0.125;
        break;
    case 4:
        this->data()[0] = -0.16666666666666666666666666666667;
        this->data()[1] =  2;
        this->data()[2] = -6.5;
        this->data()[3] =  9.3333333333333333333333333333333;
        this->data()[4] = -6.5;
        this->data()[5] =  2;
        this->data()[6] = -0.16666666666666666666666666666667;
        break;

}
break;
case 8:
switch (aOrder) {
    case 2:
        this->data()[0] =  0.011241319444444444444444444444444;
        this->data()[1] = -0.10828993055555555555555555555556;
        this->data()[2] =  0.507421875;
        this->data()[3] = -0.41037326388888888888888888888889;
        this->data()[4] = -0.41037326388888888888888888888889;
        this->data()[5] =  0.507421875;
        this->data()[6] = -0.10828993055555555555555555555556;
        this->data()[7] =  0.011241319444444444444444444444444;
        break;
    case 3:
        this->data()[0] = -0.0048177083333333333333333333333333;
        this->data()[1] =  0.064973958333333333333333333333333;
        this->data()[2] = -0.507421875;
        this->data()[3] =  1.2311197916666666666666666666667;
        this->data()[4] = -1.2311197916666666666666666666667;
        this->data()[5] =  0.507421875;
        this->data()[6] = -0.064973958333333333333333333333333;
        this->data()[7] =  0.0048177083333333333333333333333333;
        break;
    case 4:
        this->data()[0] = -0.018229166666666666666666666666667;
        this->data()[1] =  0.15364583333333333333333333333333;
        this->data()[2] = -0.3515625;
        this->data()[3] =  0.21614583333333333333333333333333;
        this->data()[4] =  0.21614583333333333333333333333333;
        this->data()[5] = -0.3515625;
        this->data()[6] =  0.15364583333333333333333333333333;
        this->data()[7] = -0.018229166666666666666666666666667;
        break;

}
break;
case 9:
switch (aOrder) {
    case 2:
        this->data()[0] = -0.0017857142857142857142857142857143;
        this->data()[1] =  0.025396825396825396825396825396825;
        this->data()[2] = -0.2;
        this->data()[3] =  1.6;
        this->data()[4] = -2.8472222222222222222222222222222;
        this->data()[5] =  1.6;
        this->data()[6] = -0.2;
        this->data()[7] =  0.025396825396825396825396825396825;
        this->data()[8] = -0.0017857142857142857142857142857143;
        break;
    case 3:
        this->data()[0] = -0.029166666666666666666666666666667;
        this->data()[1] =  0.3;
        this->data()[2] = -1.4083333333333333333333333333333;
        this->data()[3] =  2.0333333333333333333333333333333;
        this->data()[4] =  0;
        this->data()[5] = -2.0333333333333333333333333333333;
        this->data()[6] =  1.4083333333333333333333333333333;
        this->data()[7] = -0.3;
        this->data()[8] =  0.029166666666666666666666666666667;
        break;
    case 4:
        this->data()[0] =  0.029166666666666666666666666666667;
        this->data()[1] = -0.4;
        this->data()[2] =  2.8166666666666666666666666666667;
        this->data()[3] = -8.1333333333333333333333333333333;
        this->data()[4] =  11.375;
        this->data()[5] = -8.1333333333333333333333333333333;
        this->data()[6] =  2.8166666666666666666666666666667;
        this->data()[7] = -0.4;
        this->data()[8] =  0.029166666666666666666666666666667;
        break;

}
break;
case 10:
switch (aOrder) {
    case 2:
        this->data()[0] = -0.0025026351686507936507936507936508;
        this->data()[1] =  0.028759765625;
        this->data()[2] = -0.15834263392857142857142857142857;
        this->data()[3] =  0.57749565972222222222222222222222;
        this->data()[4] = -0.44541015625;
        this->data()[5] = -0.44541015625;
        this->data()[6] =  0.57749565972222222222222222222222;
        this->data()[7] = -0.15834263392857142857142857142857;
        this->data()[8] =  0.028759765625;
        this->data()[9] = -0.0025026351686507936507936507936508;
        break;
    case 3:
        this->data()[0] =  0.0008342117228835978835978835978836;
        this->data()[1] = -0.012325613839285714285714285714286;
        this->data()[2] =  0.095005580357142857142857142857143;
        this->data()[3] = -0.57749565972222222222222222222222;
        this->data()[4] =  1.33623046875;
        this->data()[5] = -1.33623046875;
        this->data()[6] =  0.57749565972222222222222222222222;
        this->data()[7] = -0.095005580357142857142857142857143;
        this->data()[8] =  0.012325613839285714285714285714286;
        this->data()[9] = -0.0008342117228835978835978835978836;
        break;
    case 4:
        this->data()[0] =  0.00458984375;
        this->data()[1] = -0.050358072916666666666666666666667;
        this->data()[2] =  0.24544270833333333333333333333333;
        this->data()[3] = -0.480078125;
        this->data()[4] =  0.28040364583333333333333333333333;
        this->data()[5] =  0.28040364583333333333333333333333;
        this->data()[6] = -0.480078125;
        this->data()[7] =  0.24544270833333333333333333333333;
        this->data()[8] = -0.050358072916666666666666666666667;
        this->data()[9] =  0.00458984375;
        break;

}
break;

}
}

// C G A B O R -----------------------------------------------------------------
template <class T>
CGaborReal<T>::CGaborReal(float aFrequency, float aAngle, float aSigma1, float aSigma2)
    : CFilter2D<T>() {
    // sqrt(2.0*log(2.0))/(2.0*NMath::Pi) = 0.18739
    float sigma1Sqr2 = aSigma1*0.18739/aFrequency;
    sigma1Sqr2 = 0.5/(sigma1Sqr2*sigma1Sqr2);
    float sigma2Sqr2 = aSigma2*0.18739/aFrequency;
    sigma2Sqr2 = 0.5/(sigma2Sqr2*sigma2Sqr2);
    float aCos = cos(aAngle);
    float aSin = sin(aAngle);
    float a = 0.6*aSigma1/aFrequency;
    float b = 0.6*aSigma2/aFrequency;
    float aXSize = fabs(a*aCos)+fabs(b*aSin);
    float aYSize = fabs(b*aCos)+fabs(a*aSin);
    this->setSize(1+2.0*floor(aXSize),1+2.0*floor(aYSize));
    this->shift(floor(aXSize),floor(aYSize));
    for (int y = this->AY(); y < this->BY(); y++)
        for (int x = this->AX(); x < this->BX(); x++) {
            float a = x*aCos+y*aSin;
            float b = y*aCos-x*aSin;
            float aGauss = exp(-sigma1Sqr2*a*a-sigma2Sqr2*b*b);
            float aHelp = 2.0*M_PI*aFrequency*(x*aCos+y*aSin);
            this->operator()(x,y) = aGauss*cos(aHelp);
        }
}

template <class T>
CGaborImaginary<T>::CGaborImaginary(float aFrequency, float aAngle, float aSigma1, float aSigma2)
    : CFilter2D<T>() {
    // sqrt(2.0*log(2.0))/(2.0*NMath::Pi) = 0.18739
    float sigma1Sqr2 = aSigma1*0.18739/aFrequency;
    sigma1Sqr2 = 0.5/(sigma1Sqr2*sigma1Sqr2);
    float sigma2Sqr2 = aSigma2*0.18739/aFrequency;
    sigma2Sqr2 = 0.5/(sigma2Sqr2*sigma2Sqr2);
    float aCos = cos(aAngle);
    float aSin = sin(aAngle);
    float a = 0.6*aSigma1/aFrequency;
    float b = 0.6*aSigma2/aFrequency;
    float aXSize = fabs(a*aCos)+fabs(b*aSin);
    float aYSize = fabs(b*aCos)+fabs(a*aSin);
    this->setSize(1+2.0*floor(aXSize),1+2.0*floor(aYSize));
    this->shift(floor(aXSize),floor(aYSize));
    for (int y = this->AY(); y < this->BY(); y++)
        for (int x = this->AX(); x < this->BX(); x++) {
            float a = x*aCos+y*aSin;
            float b = y*aCos-x*aSin;
            float aGauss = exp(-sigma1Sqr2*a*a-sigma2Sqr2*b*b);
            float aHelp = 2.0*M_PI*aFrequency*(x*aCos+y*aSin);
            this->operator()(x,y) = aGauss*sin(aHelp);
        }
}

// F I L T E R -----------------------------------------------------------------

namespace NFilter {

// 1D linear filtering ---------------------------------------------------------

template <class T>
inline void filter(Vector<T>& aVector, const CFilter<T>& aFilter) {
    Vector<T> oldVector(aVector);
    filter(oldVector,aVector,aFilter);
}

template <class T>
void filter(const Vector<T>& aVector, Vector<T>& aResult, const CFilter<T>& aFilter) {
    assert (aResult.size() == aVector.size());
    int x1 = -aFilter.A();
    int x2 = aVector.size()-aFilter.B();
    int a2Size = 2*aVector.size()-1;
    // Left rim
    for (int i = 0; i < x1; i++) {
        aResult[i] = 0;
        for (int j = aFilter.A(); j < aFilter.B(); j++)
            if (j+i < 0) aResult(i) += aFilter(j)*aVector(-1-j-i);
            else aResult(i) += aFilter(j)*aVector(j+i);
    }
    // Middle
    for (int i = x1; i < x2; i++) {
        aResult[i] = 0;
        for (int j = aFilter.A(); j < aFilter.B(); j++)
            aResult(i) += aFilter(j)*aVector(j+i);
    }
    // Right rim
    for (int i = x2; i < aResult.size(); i++) {
        aResult[i] = 0;
        for (int j = aFilter.A(); j < aFilter.B(); j++)
            if (j+i >= aVector.size()) aResult(i) += aFilter(j)*aVector(a2Size-j-i);
            else aResult(i) += aFilter(j)*aVector(j+i);
    }
}

// 2D linear filtering ---------------------------------------------------------

template <class T>
inline void filter(Matrix<T>& aMatrix, const CFilter<T>& aFilterX, const CFilter<T>& aFilterY) {
    Matrix<T> tempMatrix(aMatrix.xSize(),aMatrix.ySize());
    filter(aMatrix,tempMatrix,aFilterX,1);
    filter(tempMatrix,aMatrix,1,aFilterY);
}

template <class T>
inline void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter<T>& aFilterX, const CFilter<T>& aFilterY) {
    Matrix<T> tempMatrix(aMatrix.xSize(),aMatrix.ySize());
    filter(aMatrix,tempMatrix,aFilterX,1);
    filter(tempMatrix,aResult,1,aFilterY);
}

template <class T>
inline void filter(Matrix<T>& aMatrix, const CFilter<T>& aFilter, const int aDummy) {
    Matrix<T> tempMatrix(aMatrix.xSize(),aMatrix.ySize());
    filter(aMatrix,tempMatrix,aFilter,1);
    aMatrix = tempMatrix;
}

template <class T>
void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter<T>& aFilter, const int aDummy) {
    assert (aResult.xSize() == aMatrix.xSize() || aResult.ySize() == aMatrix.ySize());
    int x1 = -aFilter.A();
    int x2 = aMatrix.xSize()-aFilter.B();
    int a2Size = 2*aMatrix.xSize()-1;
    aResult = 0;
    for (int y = 0; y < aMatrix.ySize(); y++) {
        int aOffset = y*aMatrix.xSize();
        // Left rim
        for (int x = 0; x < x1; x++)
            for (int i = aFilter.A(); i < aFilter.B(); i++) {
                if (x+i < 0) aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset-1-x-i];
                else if (x+i >= aMatrix.xSize()) aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset+a2Size-x-i];
                else aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset+x+i];
            }
        // Center
        for (int x = x1; x < x2; x++)
            for (int i = aFilter.A(); i < aFilter.B(); i++)
                aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset+x+i];
        // Right rim
        for (int x = x2; x < aMatrix.xSize(); x++)
            for (int i = aFilter.A(); i < aFilter.B(); i++) {
                if (x+i < 0) aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset-1-x-i];
                else if (x+i >= aMatrix.xSize()) aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset+a2Size-x-i];
                else aResult.data()[aOffset+x] += aFilter[i]*aMatrix.data()[aOffset+x+i];
            }
    }
}

template <class T>
inline void filter(Matrix<T>& aMatrix, const int aDummy, const CFilter<T>& aFilter) {
    Matrix<T> tempMatrix(aMatrix.xSize(),aMatrix.ySize());
    filter(aMatrix,tempMatrix,1,aFilter);
    aMatrix = tempMatrix;
}

template <class T>
void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const int aDummy, const CFilter<T>& aFilter) {
    assert (aResult.xSize() == aMatrix.xSize() || aResult.ySize() == aMatrix.ySize());
    int y1 = -aFilter.A();
    int y2 = aMatrix.ySize()-aFilter.B();
    int a2Size = 2*aMatrix.ySize()-1;
    // Upper rim
    for (int y = 0; y < y1; y++)
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.A(); j < aFilter.B(); j++) {
                if (y+j < 0) aResult(x,y) += aFilter[j]*aMatrix(x,-1-y-j);
                else if (y+j >= aMatrix.ySize()) aResult(x,y) += aFilter[j]*aMatrix(x,a2Size-y-j);
                else aResult(x,y) += aFilter[j]*aMatrix(x,y+j);
            }
        }
    // Lower rim
    for (int y = y2; y < aMatrix.ySize(); y++)
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.A(); j < aFilter.B(); j++) {
                if (y+j < 0) aResult(x,y) += aFilter[j]*aMatrix(x,-1-y-j);
                else if (y+j >= aMatrix.ySize()) aResult(x,y) += aFilter[j]*aMatrix(x,a2Size-y-j);
                else aResult(x,y) += aFilter[j]*aMatrix(x,y+j);
            }
        }
    // Center
    for (int y = y1; y < y2; y++)
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.A(); j < aFilter.B(); j++)
                aResult(x,y) += aFilter[j]*aMatrix(x,y+j);
        }
}

template <class T>
inline void filter(Matrix<T>& aMatrix, const CFilter2D<T>& aFilter) {
    Matrix<T> tempMatrix(aMatrix.xSize(),aMatrix.ySize());
    filter(aMatrix,tempMatrix,aFilter);
    aMatrix = tempMatrix;
}

template <class T>
void filter(const Matrix<T>& aMatrix, Matrix<T>& aResult, const CFilter2D<T>& aFilter) {
  assert (aResult.xSize() == aMatrix.xSize() || aResult.ySize() == aMatrix.ySize());

    int x1 = -aFilter.AX();
    int y1 = -aFilter.AY();
    int x2 = aMatrix.xSize()-aFilter.BX();
    int y2 = aMatrix.ySize()-aFilter.BY();
    int a2XSize = 2*aMatrix.xSize()-1;
    int a2YSize = 2*aMatrix.ySize()-1;
    // Upper rim
    for (int y = 0; y < y1; y++)
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.AY(); j < aFilter.BY(); j++) {
                int tempY;
                if (y+j < 0) tempY = -1-y-j;
                else if (y+j >= aMatrix.ySize()) tempY = a2YSize-y-j;
                else tempY = y+j;
                for (int i = aFilter.AX(); i < aFilter.BX(); i++) {
                    if (x+i < 0) aResult(x,y) += aFilter(i,j)*aMatrix(-1-x-i,tempY);
                    else if (x+i >= aMatrix.xSize()) aResult(x,y) += aFilter(i,j)*aMatrix(a2XSize-x-i,tempY);
                    else aResult(x,y) += aFilter(i,j)*aMatrix(x+i,tempY);
                }
            }
        }
    // Lower rim
    for (int y = y2; y < aMatrix.ySize(); y++)
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.AY(); j < aFilter.BY(); j++) {
                int tempY;
                if (y+j < 0) tempY = -1-y-j;
                else if (y+j >= aMatrix.ySize()) tempY = a2YSize-y-j;
                else tempY = y+j;
                for (int i = aFilter.AX(); i < aFilter.BX(); i++) {
                    if (x+i < 0) aResult(x,y) += aFilter(i,j)*aMatrix(-1-x-i,tempY);
                    else if (x+i >= aMatrix.xSize()) aResult(x,y) += aFilter(i,j)*aMatrix(a2XSize-x-i,tempY);
                    else aResult(x,y) += aFilter(i,j)*aMatrix(x+i,tempY);
                }
            }
        }
    for (int y = y1; y < y2; y++) {
        // Left rim
        for (int x = 0; x < x1; x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.AY(); j < aFilter.BY(); j++) {
                for (int i = aFilter.AX(); i < aFilter.BX(); i++) {
                    if (x+i < 0) aResult(x,y) += aFilter(i,j)*aMatrix(-1-x-i,y+j);
                    else if (x+i >= aMatrix.xSize()) aResult(x,y) += aFilter(i,j)*aMatrix(a2XSize-x-i,y+j);
                    else aResult(x,y) += aFilter(i,j)*aMatrix(x+i,y+j);
                }
            }
        }
        // Right rim
        for (int x = x2; x < aMatrix.xSize(); x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.AY(); j < aFilter.BY(); j++) {
                for (int i = aFilter.AX(); i < aFilter.BX(); i++) {
                    if (x+i < 0) aResult(x,y) += aFilter(i,j)*aMatrix(-1-x-i,y+j);
                    else if (x+i >= aMatrix.xSize()) aResult(x,y) += aFilter(i,j)*aMatrix(a2XSize-x-i,y+j);
                    else aResult(x,y) += aFilter(i,j)*aMatrix(x+i,y+j);
                }
            }
        }
    }
    // Center
    for (int y = y1; y < y2; y++)
        for (int x = x1; x < x2; x++) {
            aResult(x,y) = 0;
            for (int j = aFilter.AY(); j < aFilter.BY(); j++)
                for (int i = aFilter.AX(); i < aFilter.BX(); i++)
                    aResult(x,y) += aFilter(i,j)*aMatrix(x+i,y+j);
        }
}

// boxfilterX
template <class T>
inline void boxFilterX(Matrix<T>& aMatrix, int aWidth) {
    Matrix<T> aTemp(aMatrix);
    boxFilterX(aTemp,aMatrix,aWidth);
}

template <class T>
void boxFilterX(const Matrix<T>& aMatrix, Matrix<T>& aResult, int aWidth) {
    if (aWidth & 1 == 0) aWidth += 1;
    T invWidth = 1.0/aWidth;
    int halfWidth = (aWidth >> 1);
    int aRight = halfWidth;
    int aDiff = 0;
    if (aRight >= aMatrix.xSize()) {
        aDiff = aMatrix.xSize()-1-aRight;
        aRight = aMatrix.xSize()-1;
    }
    for (int y = 0; y < aMatrix.ySize(); y++) {
        int aOffset = y*aMatrix.xSize();
        // Initialize
        T aSum = halfWidth*aMatrix.data()[aOffset];
        for (int x = 0; x <= aRight; x++)
            aSum += aMatrix.data()[aOffset+x];
        aSum += aDiff*aMatrix.data()[aOffset+aMatrix.xSize()-1];
        // Shift
        for (int x = 0; x < aMatrix.xSize(); x++) {
            aResult.data()[aOffset+x] = aSum*invWidth;
            if (x-halfWidth < 0) aSum -= aMatrix.data()[aOffset];
            else aSum -= aMatrix.data()[aOffset+x-halfWidth];
            if (x+halfWidth+1 >= aMatrix.xSize()) aSum += aMatrix.data()[aOffset+aMatrix.xSize()-1];
            else aSum += aMatrix.data()[aOffset+x+halfWidth+1];
        }
    }
}

// boxfilterY
template <class T>
inline void boxFilterY(Matrix<T>& aMatrix, int aWidth) {
    Matrix<T> aTemp(aMatrix);
    boxFilterY(aTemp,aMatrix,aWidth);
}

template <class T>
void boxFilterY(const Matrix<T>& aMatrix, Matrix<T>& aResult, int aWidth) {
    if (aWidth & 1 == 0) aWidth += 1;
    float invWidth = 1.0/aWidth;
    int halfWidth = (aWidth >> 1);
    int aBottom = halfWidth;
    int aDiff = 0;
    if (aBottom >= aMatrix.ySize()) {
        aDiff = aMatrix.ySize()-1-aBottom;
        aBottom = aMatrix.ySize()-1;
    }
    for (int x = 0; x < aMatrix.xSize(); x++) {
        // Initialize
        float aSum = halfWidth*aMatrix(x,0);
        for (int y = 0; y <= aBottom; y++)
            aSum += aMatrix(x,y);
        aSum += aDiff*aMatrix(x,aMatrix.ySize()-1);
        // Shift
        for (int y = 0; y < aMatrix.ySize(); y++) {
            aResult(x,y) = aSum*invWidth;
            if (y-halfWidth < 0) aSum -= aMatrix(x,0);
            else aSum -= aMatrix(x,y-halfWidth);
            if (y+halfWidth+1 >= aMatrix.ySize()) aSum += aMatrix(x,aMatrix.ySize()-1);
            else aSum += aMatrix(x,y+halfWidth+1);
        }
    }
}
}

template <class T>
inline void osher(const Matrix<T>& aData, Matrix<T>& aResult, int aIterations) {
    aResult = aData;
    osher(aResult,aIterations);
}



#endif

