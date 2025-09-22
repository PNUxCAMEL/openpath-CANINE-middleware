//
// Created by hs on 22. 11. 1.
//

#ifndef RAISIM_FILTER_HPP
#define RAISIM_FILTER_HPP

#include <iostream>
#include <cassert>
#include <initializer_list>
#include <algorithm>
#include <Eigen/Dense>


namespace CamelFilter
{
    class LPF
    {
    public:
        LPF(double dt, double fc, int numFilter);
        void GetFilteredVar(double* FilteredData, const double* rawData);

    private:
        void doFiltering(void);

        bool mbIsFirstRun;
        const double mDT;
        const int mNumFilter;
        const double mCutoffFreq;
        const double mAlpha;
        Eigen::VectorXd mInputData;
        Eigen::VectorXd mPreviousData;
        Eigen::VectorXd mFilteredData;
    };

    class MAF
    {
    public:
        MAF(std::initializer_list<int> filterSizes, int numFilter);
        MAF(int FilterSize);
        void GetFilteredVar(double* filteredData, const double* rawData);

    private:
        void doFiltering();

        const double mNumofFilter;
        Eigen::VectorXi mFilterSize;
        Eigen::MatrixXd mData;
        Eigen::VectorXd mInputData;
        Eigen::VectorXd mFilteredData;
    };

    template <typename T>
    class EWMA
    {
    public:
        // 생성자: 커트오프 주파수 [Hz]와 샘플링 시간 [s]로 λ 자동 계산
        EWMA(double dt, double cutoff_freq)
            : lambda_(computeLambda(cutoff_freq, dt)),
              one_minus_lambda_(1.0 - lambda_),
              initialized_(false)
        {
        }

        const T& update(const T& x_k)
        {
            if (!initialized_)
            {
                mu_ = x_k;
                sigma2_.setZero(x_k.rows(), x_k.cols());
                initialized_ = true;
                return mu_;
            }
            T mu_prev = mu_;
            mu_ = one_minus_lambda_ * mu_ + lambda_ * x_k;
            T diff = x_k - mu_;
            sigma2_ = one_minus_lambda_ * sigma2_ + lambda_ * (diff.array() * diff.array()).matrix();
            return mu_;
        }

        const T& mean() const { return mu_; }
        const T& var() const { return sigma2_; }

        T stddev() const
        {
            if constexpr (std::is_floating_point_v<T>)
                return std::sqrt(sigma2_);
            else
                return sigma2_.cwiseSqrt();
        }

    private:
        double lambda_, one_minus_lambda_;
        bool initialized_;
        T mu_;
        T sigma2_;

        static double computeLambda(double fc, double dt)
        {
            constexpr double TWO_PI = 6.28318530718;
            double wc_dt = TWO_PI * fc * dt;
            return wc_dt / (wc_dt + 1.0);
        }
    };
}


#endif //RAISIM_FILTER_HPP
