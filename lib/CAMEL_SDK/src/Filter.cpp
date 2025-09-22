//
// Created by hs on 22. 11. 1.
//
#include "CAMEL_SDK/MathTools/Filter.hpp"

CamelFilter::LPF::LPF(double dt, double fc, int numFilter)
    : mbIsFirstRun(true)
      , mDT(dt)
      , mCutoffFreq(fc)
      , mNumFilter(numFilter)
      , mAlpha(2 * 3.141592 * mCutoffFreq * mDT / (2 * 3.141592 * mCutoffFreq * mDT + 1))
{
    mInputData.resize(mNumFilter);
    mPreviousData.resize(mNumFilter);
    mFilteredData.resize(mNumFilter);
}

void CamelFilter::LPF::GetFilteredVar(double* FilteredData, const double* rawData)
{
    std::copy_n(rawData, mNumFilter, mInputData.data());
    doFiltering();
    std::copy_n(mFilteredData.data(), mNumFilter, FilteredData);
}

void CamelFilter::LPF::doFiltering(void)
{
    if (mbIsFirstRun == true)
    {
        mPreviousData = mInputData;
        mbIsFirstRun = false;
    }
    mFilteredData = mAlpha * mInputData + (1 - mAlpha) * mPreviousData;
    mPreviousData = mFilteredData;
}

CamelFilter::MAF::MAF(std::initializer_list<int> filterSizes, int numFilter)
    : mNumofFilter(numFilter)
{
    // filterSizes와 NumofFilter의 크기가 일치하는지 확인
    assert(filterSizes.size() == mNumofFilter && "Filter size and number of filters must match.");
    mInputData = Eigen::VectorXd::Zero(mNumofFilter);
    mFilteredData = Eigen::VectorXd::Zero(mNumofFilter);
    mFilterSize = Eigen::VectorXi::Map(filterSizes.begin(), filterSizes.size());
    int maxSize = *std::max_element(filterSizes.begin(), filterSizes.end());
    mData = Eigen::MatrixXd::Zero(mNumofFilter, maxSize);
}

CamelFilter::MAF::MAF(int FilterSize)
    : mNumofFilter(1)
{
    // 채널 수만큼 벡터를 만들고, 각 채널의 윈도 크기를 FilterSize로 설정
    mFilterSize = Eigen::VectorXi::Constant(mNumofFilter, FilterSize);

    mInputData    = Eigen::VectorXd::Zero(mNumofFilter);
    mFilteredData = Eigen::VectorXd::Zero(mNumofFilter);
    mData         = Eigen::MatrixXd::Zero(mNumofFilter, FilterSize);
}


void CamelFilter::MAF::GetFilteredVar(double* filteredData, const double* rawData)
{
    std::copy_n(rawData, mNumofFilter, mInputData.data());
    doFiltering();
    std::copy_n(mFilteredData.data(), mNumofFilter, filteredData);
}

void CamelFilter::MAF::doFiltering()
{
    for (int idx = 0; idx < mNumofFilter; ++idx)
    {
        const int W = mFilterSize[idx];
        // 방어
        if (W <= 0) { mFilteredData[idx] = mInputData[idx]; continue; }
        if (W == 1) { mData(idx,0) = mInputData[idx]; mFilteredData[idx] = mData(idx,0); continue; }

        // 좌로 한 칸씩 밀기 (OOB 방지)
        for (int j = 0; j < W - 1; ++j) {
            mData(idx, j) = mData(idx, j + 1);
        }
        // 최신 샘플 삽입
        mData(idx, W - 1) = mInputData[idx];

        // 평균
        double sum = 0.0;
        for (int j = 0; j < W; ++j) sum += mData(idx, j);
        mFilteredData[idx] = sum / static_cast<double>(W);
    }
}
