
#pragma once

#include "GenICam/Frame.h"
#include "Memory/Block.h"
#include "GenICam/PixelType.h"
#include "Media/ImageConvert.h"
#include "GenICam/PixelType.h"
#include <assert.h>
#include "Infra/Time.h"

using namespace Dahua::GenICam;

class FrameBuffer
{
private:
    uint8_t* Buffer_;

    int Width_;

    int Height_;

    int PaddingX_;

    int PaddingY_;

    int DataSize_;

    int PixelFormat_;

    uint64_t TimeStamp_;

public:
    FrameBuffer(Dahua::GenICam::CFrame const& frame)
    {
        if (frame.getImageSize() > 0)
        {
            if (frame.getImagePixelFormat() == Dahua::GenICam::gvspPixelMono8)
            {
                Buffer_ = new(std::nothrow) uint8_t[frame.getImageSize()];
            }
            else
            {
                Buffer_ = new(std::nothrow) uint8_t[frame.getImageWidth() * frame.getImageHeight() * 3];
            }
            if (Buffer_)
            {
                Width_ = frame.getImageWidth();
                Height_ = frame.getImageHeight();
                PaddingX_ = frame.getImagePadddingX();
                PaddingY_ = frame.getImagePadddingY();
                DataSize_ = frame.getImageSize();
                PixelFormat_ = frame.getImagePixelFormat();
            }
        }
    }

    ~FrameBuffer()
    {
        if (Buffer_ != NULL)
        {
            delete[] Buffer_;
            Buffer_ = NULL;
        }
    }

    bool Valid()
    {
        if (NULL != Buffer_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int Width()
    {
        return Width_;
    }

    int Height()
    {
        return Height_;
    }

    int PaddingX()
    {
        return PaddingX_;
    }

    int PaddingY()
    {
        return PaddingY_;
    }

    int DataSize()
    {
        return DataSize_;
    }

    uint64_t PixelFormat()
    {
        return PixelFormat_;
    }

    uint64_t TimeStamp()
    {
        return TimeStamp_;
    }

    void setWidth(uint32_t iWidth)
    {
        Width_ = iWidth;
    }

    void setPaddingX(uint32_t iPaddingX)
    {
        PaddingX_ = iPaddingX;
    }

    void setPaddingY(uint32_t iPaddingX)
    {
        PaddingY_ = iPaddingX;
    }

    void setHeight(uint32_t iHeight)
    {
        Height_ = iHeight;
    }

    void setDataSize(int dataSize)
    {
        DataSize_ = dataSize;
    }

    void setPixelFormat(uint32_t pixelFormat)
    {
        PixelFormat_ = pixelFormat;
    }

    void setTimeStamp(uint64_t timeStamp)
    {
        TimeStamp_ = timeStamp;
    }

    uint8_t* bufPtr()
    {
        return Buffer_;
    }

};
typedef Dahua::Memory::TSharedPtr<FrameBuffer> FrameBufferSPtr;

extern bool ConvertImage(const Dahua::GenICam::CFrame& input, FrameBufferSPtr& output);

static uint32_t gFormatTransferTbl[] =
        {
                // Mono Format
                gvspPixelMono1p,
                gvspPixelMono8,
                gvspPixelMono10,
                gvspPixelMono10Packed,
                gvspPixelMono12,
                gvspPixelMono12Packed,

                // Bayer Format
                gvspPixelBayRG8,
                gvspPixelBayGB8,
                gvspPixelBayBG8,
                gvspPixelBayRG10,
                gvspPixelBayGB10,
                gvspPixelBayBG10,
                gvspPixelBayRG12,
                gvspPixelBayGB12,
                gvspPixelBayBG12,
                gvspPixelBayRG10Packed,
                gvspPixelBayGB10Packed,
                gvspPixelBayBG10Packed,
                gvspPixelBayRG12Packed,
                gvspPixelBayGB12Packed,
                gvspPixelBayBG12Packed,
                gvspPixelBayRG16,
                gvspPixelBayGB16,
                gvspPixelBayBG16,
                gvspPixelBayRG10p,
                gvspPixelBayRG12p,

                gvspPixelMono1c,

                // RGB Format
                gvspPixelRGB8,
                gvspPixelBGR8,

                // YVR Format
                gvspPixelYUV411_8_UYYVYY,
                gvspPixelYUV422_8_UYVY,
                gvspPixelYUV422_8,
                gvspPixelYUV8_UYV,
        };
#define gFormatTransferTblLen   sizeof(gFormatTransferTbl)/sizeof(gFormatTransferTbl[0])

struct ImplData
{
    int width;
    int height;
};

static int32_t findMatchCode(uint32_t code)
{
    for (int i = 0; i < gFormatTransferTblLen; ++i)
    {
        if (gFormatTransferTbl[i] == code)
        {
            return i;
        }
    }
    return -1;
}

static void freeMem(uint8_t* mem)
{
    if (NULL != mem)
        ::free(mem);
}


/* ת�뺯�� */
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
