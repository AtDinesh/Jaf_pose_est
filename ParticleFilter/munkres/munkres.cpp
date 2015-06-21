#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include "munkres.h"

using namespace Eigen;

/*
 * reduce
 * reduces matrix based on row and column minimums
 */
inline void reduce(MatrixXd& m)
{
    // subtract row minimum from each row
    const int rows = m.rows();
    for (int i=0; i<rows; i++)
    {
        VectorXd rMinusMin(m.cols());
        rMinusMin.fill(-m.row(i).minCoeff());
        m.row(i) += rMinusMin;
    }
}

/*
 * hasMark
 * if there is a starred/primed zero in the given row/col, returns it's index
 * else, returns -1
 */
inline int hasMark(const VectorXd& v)
{
    const int size = v.size();
    for (int i=0; i<size; i++)
        if (v(i))
            return i;
    return -1;
}

/* Step 3
 Cover all columns that have a starred zero
 If the number of columns with starred zeroes equals the matrix
 dimensions, we are done! Otherwise, move on to step 4.
*/
inline void step3(const MatrixXd& n, const MatrixXd& stars, VectorXd& colCover)
{
    const int cols = n.cols();
    for (int j=0; j<cols; j++)
    {
        const VectorXd col = stars.col(j);
        if (hasMark(col) >= 0)
           colCover(j) = 1;
    }
}

/*
 * swapStarsAndPrimes
 * Swap stars and primes based on step 5 of Hungarian algorithm
 * Z0 is uncovered primed zero we've found
 * Z1 is the stared zero in the column of Z0 (if any)
 * Z2 is the primed zero in the row of Z1 (will always be one)
 * ...continue series until we reach a primed zero with no starred zero in its column
 * Unstar each starred zero, star each primed zero, erase all primes and uncover every line in the matrix
 */
inline void swapStarsAndPrimes(const int i, const int j, MatrixXd& stars, MatrixXd& primes) {
    int primeRow = i;
    int primeCol = j;

    bool done = false;
    while (!done) {
        // find row index of row that has a 0* in the same col as the current 0'

        int starInPrimeColRow=-1;
        if (primeCol!=-1)
        {
            const VectorXd col = stars.col(primeCol);
            starInPrimeColRow = hasMark(col);
        }

        if (starInPrimeColRow < 0 && primeCol != -1 && primeRow != -1)
        {
            // star the prime we're looking at
            primes(primeRow, primeCol) = 0;
            stars(primeRow, primeCol) = 1;
            done = true;
        } else if (starInPrimeColRow >= 0)
        {
            // find which col has a 0' in the same row as z1
            const VectorXd row = primes.row(starInPrimeColRow);

            // star first primed zero
            primes(primeRow, primeCol) = 0;
            stars(primeRow, primeCol) = 1;

            // unstar starred zero
            stars(starInPrimeColRow, primeCol) = 0;

            // set index of last prime, will check it's column for 0*s next
            primeRow = starInPrimeColRow;
            primeCol = hasMark(row);
        } else
            done = true;
    }
    // clear primes
    primes.fill(0);
}

void Munkres(const MatrixXd& costs, MatrixXd& result)
{
    // Step 0:  Create an nxm  matrix called the cost matrix in which each element represents the cost
    // of assigning one of n workers to one of m jobs.  Rotate the matrix so that there are at least as
    // many columns as rows and let k=min(n,m).
    const bool costsWasTransposed = (costs.rows() > costs.cols());
    MatrixXd n;

    if (costsWasTransposed)
        n.noalias() = costs.transpose();
    else
        n.noalias() = costs;

    const int nrows = n.rows(), ncols = n.cols();

    MatrixXd stars(nrows, ncols); // matrix for storing our "starred" 0s (0*)
    stars.fill(0.);
    MatrixXd primes(nrows, ncols); // matrix for storing our "primed" 0s (0')
    primes.fill(0.);
    VectorXd rowCover(nrows); // keep track of which rows are "covered"
    rowCover.fill(0.);
    VectorXd colCover(ncols); // keep track of which columns are "covered"
    colCover.fill(0.);

    // to do maximization rather than minimization, we have to
    // transform the matrix by subtracting every value from the maximum
    MatrixXd maxMat(nrows, ncols);
    maxMat.fill(n.maxCoeff());
    n.noalias() = maxMat - n;

    // Step 1:  For each row of the matrix, find the smallest element and subtract it from every element
    // in its row.
    reduce(n);

    // Step 2:  Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or column,
    // star Z. Repeat for each element in the matrix.
    for (int i=0; i<nrows; i++)
        for (int j=0; j<ncols; j++)
            if (n(i,j) == 0 && !rowCover(i) && !colCover(j))
            {
                stars(i,j) = 1;
                rowCover(i) = 1;
                colCover(j) = 1;
            }
    // covers need to be cleared for following steps
    rowCover.fill(0.);
    colCover.fill(0.);

    bool computationFinished = false;

    while(!computationFinished)
    {
        // Step 3:  Cover each column containing a starred zero.  If K columns are covered, the starred zeros
        // describe a complete set of unique assignments.  In this case, Go to DONE, otherwise, Go to Step 4.
        step3(n,stars,colCover);
        if (colCover.sum() == nrows)
        {
            if (costsWasTransposed)
                result.noalias() = stars.transpose();
            else
                result.noalias() = stars;
            computationFinished = true;
        }

        if (!computationFinished)
        {
            bool skipToNextStep = false; // If at step 4 there are no starred zero in the row

            while(!skipToNextStep)
            {
                // Step 4:  Find a noncovered zero and prime it.  If there is no starred zero in the row containing
                // this primed zero, Go to Step 5.  Otherwise, cover this row and uncover the column containing the
                // starred zero. Continue in this manner until there are no uncovered zeros left. Save the smallest
                // uncovered value and Go to Step 6.
                for (int i=0; i<nrows; i++)
                {
                    for (int j=0; j<ncols; j++)
                    {
                        if (n(i,j) == 0 && !rowCover(i) && !colCover(j))
                        {
                            primes(i,j) = 1;
                            // if no starred zero in the row...
                            VectorXd row = stars.row(i);
                            if (hasMark(row) < 0)
                            {
                                // Step 5
                                // swap stars and primes
                                swapStarsAndPrimes(i, j, stars, primes);

                                // clear lines
                                rowCover.fill(0.);
                                colCover.fill(0.);

                                skipToNextStep = true;
                            } else {
                                // cover row
                                rowCover(i) = 1;

                                // uncover column of the starred zero in the same row
                                colCover(hasMark(row)) = 0;
                            }
                        }
                    }
                }

                if (!skipToNextStep)
                {
                    // Step 6:  Add the value found in Step 4 to every element of each covered row, and subtract it from every
                    // element of each uncovered column.  Return to Step 4 without altering any stars, primes, or covered lines.
                    float min = 1000000; // a bit hacky...
                    for (int i=0; i<nrows; i++)
                        for (int j=0; j<ncols; j++)
                            if (!rowCover(i) && !colCover(j) && n(i,j) < min)
                                min = n(i,j);

                    // Subtract minimum from uncovered elements, add it to elements covered twice
                    for (int i=0; i<nrows; i++)
                        for (int j=0; j<ncols; j++)
                            if (!rowCover(i) && !colCover(j))
                            {
                                n(i,j) -= min;
                            } else if (rowCover(i) && colCover(j))
                                n(i,j) += min;
                }
            }
        }
    }
}
