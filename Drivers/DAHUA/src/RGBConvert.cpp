//
// Created by root on 2021/1/17.
//
#include "Media/RGBConvert.h"

bool ConvertImage(const Dahua::GenICam::CFrame& input, FrameBufferSPtr& output)
{
    int idx = findMatchCode((input.getImagePixelFormat()));
    if (idx < 0)
    {
        return false;
    }

    FrameBufferSPtr PtrFrameBuffer(new FrameBuffer(input));
    if (!PtrFrameBuffer)
    {
        perror("PtrFrameBuffer is null.\n");
        return false;
    }

    /* Mono8����ת��ֱ��Դ������ʾ */
    if (PtrFrameBuffer->PixelFormat() == gvspPixelMono8)
    {
        memcpy(PtrFrameBuffer->bufPtr(), input.getImage(), input.getImageSize());
    }
    else
    {
        uint8_t* pSrcData = new(std::nothrow) uint8_t[input.getImageSize()];
        if (pSrcData)
        {
            memcpy(pSrcData, input.getImage(), input.getImageSize());
        }
        else
        {
            perror("m_pSrcData is null.\n");
            return false;
        }

        int dstDataSize = 0;
        IMGCNV_SOpenParam openParam;
        openParam.width = PtrFrameBuffer->Width();
        openParam.height = PtrFrameBuffer->Height();
        openParam.paddingX = PtrFrameBuffer->PaddingX();
        openParam.paddingY = PtrFrameBuffer->PaddingY();
        openParam.dataSize = PtrFrameBuffer->DataSize();
        openParam.pixelForamt = PtrFrameBuffer->PixelFormat();

        IMGCNV_EErr status = IMGCNV_ConvertToBGR24(pSrcData, &openParam, PtrFrameBuffer->bufPtr(), &dstDataSize);
        if (IMGCNV_SUCCESS != status)
        {
            delete[] pSrcData;
            perror("IMGCNV_open is failed!\n");
            return false;
        }

        delete[] pSrcData;
    }

    output = PtrFrameBuffer;
    return true;
}
