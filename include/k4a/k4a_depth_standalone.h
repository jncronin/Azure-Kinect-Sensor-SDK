#ifndef K4A_DEPTH_STANDALONE_H
#define K4A_DEPTH_STANDALONE_H

#ifdef __cplusplus
extern "C" {
#endif

void DeinitNFOVUnbinnedCalculation();
void InitNFOVUnbinnedCalculation();
void RunNFOVUnbinnedCalculation(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin
#ifdef __cplusplus
    = 1
#endif
    , int ybin
#ifdef __cplusplus
    = 1
#endif
);

void DeinitWFOVBinnedCalculation();
void InitWFOVBinnedCalculation();
void RunWFOVBinnedCalculation(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin
#ifdef __cplusplus
    = 1
#endif
    , int ybin
#ifdef __cplusplus
    = 1
#endif
);

#ifdef __cplusplus
}
#endif

#endif
