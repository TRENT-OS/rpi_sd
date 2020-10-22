/**
* Copyright (C) 2020, Hensoldt Cyber GmbH
*/
#include <circle/emmc.h>
#include "OS_Error.h"
#include "OS_Dataport.h"
#include "LibDebug/Debug.h"
#include "environment.h"
#include <circleos.h>
#include <circle/bcm2837_sdhost.h>

#include <string.h>
#include <stdlib.h>

// #include <sel4/sel4.h>

#include <camkes.h>

boolean init_ok = false;
OS_Dataport_t port_storage = OS_DATAPORT_ASSIGN(storage_port);

static
bool
isValidEMMCArea(
    off_t const offset,
    off_t const size)
{
    off_t const end = offset + size;
    return ( (offset >= 0) && (size >= 0) && (end >= offset) && (end <= emmc_capacity()));
}

// as interrupts can not be connected in post_init, we call this function the first time we actually need it
void emmc_init(void)
{
    int ret = 0;
    Debug_LOG_INFO("SD card initialization.");
    ret = emmc_initialize();
    if(!ret)
    {
        Debug_LOG_ERROR("emmc_initialize() failed");
        return;
    }
    init_ok = true;
    Debug_LOG_INFO("SD card initialization successful.");
}

void post_init(void)
{
    init_ok = false;
    Debug_LOG_INFO("EMMCDevice_init");
    int ret = 0;
#ifndef USE_SDHOST
    ret = EMMCDevice(emmcBaseReg);
#else 
    ret = EMMCDevice(sdBaseReg);
#endif
    if(!ret)
    {
        Debug_LOG_ERROR("EMMCDevice() failed");
        return;
    }
    Debug_LOG_INFO("post_init() done");
}


/* EMMC interrupt handler -----------------------------------------------------------------*/
void emmcBaseIrq_handle(void) {
#ifndef USE_SDHOST
    HandleInterrupts();
#else
    Debug_LOG_INFO("Using SDHost mode. -> no EMMC IRQ handle configured.");
#endif

	int error = emmcBaseIrq_acknowledge();

    if(error != 0)
	{
		Debug_LOG_ERROR("Failed to acknowledge the interrupt!");
	}
}

/* SD interrupt handler -----------------------------------------------------------------*/
void sdBaseIrq_handle(void) {
#ifndef USE_SDHOST
    Debug_LOG_INFO("Using EMMC mode. -> no SDHost IRQ handle configured.");
#else 
    irq_stub(NULL);
#endif

	int error = sdBaseIrq_acknowledge();

    if(error != 0)
	{
		Debug_LOG_ERROR("Failed to acknowledge the interrupt!");
	}
}

OS_Error_t
__attribute__((__nonnull__))
storage_rpc_write(
    off_t offset,
    size_t  size,
    size_t* written)
{
    // set defaults
    *written = 0;

    Debug_LOG_DEBUG(
        "EMMC write: offset %jd (0x%jx), size %zu (0x%zx)",
        offset, offset, size, size);

    if(!init_ok)
    {
        emmc_init();
    }

    size_t dataport_size = OS_Dataport_getSize(port_storage);
    if (size > dataport_size)
    {
        // the client did a bogus request, it knows the data port size and
        // never ask for more data
        Debug_LOG_ERROR(
            "size %zu exceeds dataport size %zu",
            size,
            dataport_size );

        return OS_ERROR_INVALID_PARAMETER;
    }

    if (!isValidEMMCArea(offset, size))
    {
        Debug_LOG_ERROR(
            "write area at offset %jd with size %zu out of bounds",
            offset, size);
        return OS_ERROR_OUT_OF_BOUNDS;
    }

    const void* buffer = OS_Dataport_getBuf(port_storage);

    int ret = 0;
    int bytesWritten = 0;
    if (size > 0)
    {
        uint8_t block[emmc_block_size()];
        memset(block,0,emmc_block_size());

        uint32_t sector = offset / emmc_block_size();
        //read first block, adjust according bytes and write block back
        // ret = disk_read(&(ctx.spi_sd_ctx),block,sector,1);
        ret = DoRead(block,emmc_block_size(),sector);
        if (ret < 0){
            Debug_LOG_ERROR( "disk_read() failed => emmc_write() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                offset, offset, bytesWritten, bytesWritten, ret);
            return OS_ERROR_GENERIC;
        }
        size_t nbr_of_bytes = size <= (emmc_block_size() - (offset - sector * emmc_block_size())) ? size : (emmc_block_size() - (offset - sector * emmc_block_size()));
        memcpy(block + (offset - sector * emmc_block_size()),buffer,nbr_of_bytes);
        // ret = disk_write(&(ctx.spi_sd_ctx),block,sector,1);
        ret = DoWrite(block,emmc_block_size(),sector);
        if (ret < 0){
            Debug_LOG_ERROR( "disk_write() failed => emmc_write() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                offset, offset, bytesWritten, bytesWritten, ret);
            return OS_ERROR_GENERIC;
        }

        bytesWritten += nbr_of_bytes;
        size -= nbr_of_bytes;
        buffer += nbr_of_bytes;

        //write the remaining blocks
        while (size > emmc_block_size())
        {
            memcpy(block,buffer,emmc_block_size());
            // ret = disk_write(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoWrite(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_write() failed => SPISD_write() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesWritten, bytesWritten, ret);
                return OS_ERROR_GENERIC;
            }
            bytesWritten += emmc_block_size();
            size -= emmc_block_size();
            buffer += emmc_block_size();
        }

        if (size > 0)
        {
            //read last block, adjust according bytes and write block back
            // ret = disk_read(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoRead(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_read() failed => emmc_write() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesWritten, bytesWritten, ret);
                return OS_ERROR_GENERIC;
            }
            memcpy(block,buffer,size);
            // ret = disk_write(&(ctx.spi_sd_ctx),block,sector,1);
            ret = DoWrite(block,emmc_block_size(),sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_write() failed => emmc_write() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesWritten, bytesWritten, ret);
                return OS_ERROR_GENERIC;
            }
            bytesWritten += size;
            size -= size;
        }
    }

    *written = bytesWritten;

    return OS_SUCCESS;
}

OS_Error_t
__attribute__((__nonnull__))
storage_rpc_read(
    off_t offset,
    size_t  size,
    size_t* read)
{
    *read = 0;

    Debug_LOG_DEBUG(
        "EMMC read: offset %jd (0x%jx), size %zu (0x%zx)",
        offset, offset, size, size);
    
    if(!init_ok)
    {
        emmc_init();
    }

    size_t dataport_size = OS_Dataport_getSize(port_storage);
    if (size > dataport_size)
    {
        // the client did a bogus request, it knows the data port size and
        // never ask for more data
        Debug_LOG_ERROR(
            "size %zu exceeds dataport size %zu",
            size,
            dataport_size );

        return OS_ERROR_INVALID_PARAMETER;
    }

    if (!isValidEMMCArea(offset, size))
    {
        Debug_LOG_ERROR(
            "read area at offset %jd with size %zu out of bounds",
            offset, size);
        return OS_ERROR_OUT_OF_BOUNDS;
    }

    int ret = 0;
    int bytesRead = 0;

    if (size > 0)
    {
        uint8_t block[emmc_block_size()];
        memset(block,0,emmc_block_size());
        
        uint32_t sector = offset / emmc_block_size();
        
        //read first block and copy according bytes to dataport
        // ret = disk_read(&(ctx.spi_sd_ctx),block,sector,1);
        ret = DoRead(block,emmc_block_size(),sector);
        if (ret < 0){
            Debug_LOG_ERROR( "disk_read() failed => emmc_read() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                offset, offset, bytesRead, bytesRead, ret);
            return OS_ERROR_GENERIC;
        }
        size_t nbr_of_bytes = size <= (emmc_block_size() - (offset - sector * emmc_block_size())) ? size : (emmc_block_size() - (offset - sector * emmc_block_size()));
        memcpy(OS_Dataport_getBuf(port_storage),block + (offset - sector * emmc_block_size()),nbr_of_bytes);
   
        bytesRead += nbr_of_bytes;
        size -= nbr_of_bytes;
    
        //read the remaining blocks and copy into dataport 
        while (size > emmc_block_size())
        {
            // ret = disk_read(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoRead(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_read() failed => emmc_read() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesRead, bytesRead, ret);
                return OS_ERROR_GENERIC;
            }
            memcpy(OS_Dataport_getBuf(port_storage) + bytesRead,block,emmc_block_size());
            bytesRead += emmc_block_size();
            size -= emmc_block_size();
        }

        if (size > 0)
        {
            //read last block and copy according bytes into the dataport
            // ret = disk_read(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoRead(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_read() failed => emmc_read() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesRead, bytesRead, ret);
                return OS_ERROR_GENERIC;
            }
            memcpy(OS_Dataport_getBuf(port_storage) + bytesRead,block,size);
            bytesRead += size;
            size -= size;
        }
    }
    
    *read = bytesRead;

    return OS_SUCCESS;
}

OS_Error_t
__attribute__((__nonnull__))
storage_rpc_erase(
    off_t offset,
    off_t size,
    off_t* erased)
{
    *erased = 0;

    Debug_LOG_DEBUG(
        "SPI erase: offset %jd (0x%jx), size %jd (0x%jx)",
        offset, offset, size, size);

    if(!init_ok)
    {
        emmc_init();
    }

    size_t dataport_size = OS_Dataport_getSize(port_storage);
    if (size > dataport_size)
    {
        // the client did a bogus request, it knows the data port size and
        // never ask for more data
        Debug_LOG_ERROR(
            "size %lld exceeds dataport size %zu",
            size,
            dataport_size );

        return OS_ERROR_INVALID_PARAMETER;
    }

    if (!isValidEMMCArea(offset, size))
    {
        Debug_LOG_ERROR(
            "erase area at offset %jd with size %jd out of bounds",
            offset, size);
        return OS_ERROR_OUT_OF_BOUNDS;
    }

    int ret = 0;
    size_t bytesErased = 0;

    if(size > 0){
        uint8_t block[emmc_block_size()];
        memset(block,0,emmc_block_size());
        
        uint32_t sector = offset / emmc_block_size();
        //read first block, adjust according bytes and erase block
        // ret = disk_read(&(ctx.spi_sd_ctx),block,sector,1);
        ret = DoRead(block,emmc_block_size(),sector);
        if (ret < 0){
            Debug_LOG_ERROR( "disk_read() failed => emmc_erase() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                offset, offset, bytesErased, bytesErased, ret);
            return OS_ERROR_GENERIC;
        }
        size_t nbr_of_bytes = size <= (emmc_block_size() - (offset - sector * emmc_block_size())) ? size : (emmc_block_size() - (offset - sector * emmc_block_size()));
        memset(block + (offset - sector * emmc_block_size()),0xFF,nbr_of_bytes);
        // ret = disk_write(&(ctx.spi_sd_ctx),block,sector,1);
        ret = DoWrite(block,emmc_block_size(),sector);
        if (ret < 0){
            Debug_LOG_ERROR( "disk_write() failed => emmc_erase() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                offset, offset, bytesErased, bytesErased, ret);
            return OS_ERROR_GENERIC;
        }
        
        bytesErased += nbr_of_bytes;
        size -= nbr_of_bytes;
        
        //erase the remaining blocks
        while (size > emmc_block_size())
        {
            memset(block,0xFF,emmc_block_size());
            // ret = disk_write(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoWrite(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_write() failed => emmc_erase() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesErased, bytesErased, ret);
                return OS_ERROR_GENERIC;
            }
            bytesErased += emmc_block_size();
            size -= emmc_block_size();
        }

        if (size > 0)
        {
            //read last block, adjust according bytes and erase block
            // ret = disk_read(&(ctx.spi_sd_ctx),block,++sector,1);
            ret = DoRead(block,emmc_block_size(),++sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_read() failed => emmc_erase() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesErased, bytesErased, ret);
                return OS_ERROR_GENERIC;
            }
            memset(block,0xFF,size);
            // ret = disk_write(&(ctx.spi_sd_ctx),block,sector,1);
            ret = DoWrite(block,emmc_block_size(),sector);
            if (ret < 0){
                Debug_LOG_ERROR( "disk_write() failed => emmc_erase() failed, offset %jd (0x%jx), size %zu (0x%zx), code %d",
                                 offset, offset, bytesErased, bytesErased, ret);
                return OS_ERROR_GENERIC;
            }
            bytesErased += size;
            size -= size;
        }
    }
    
    *erased = bytesErased;

    return OS_SUCCESS;
}

OS_Error_t
__attribute__((__nonnull__))
storage_rpc_getSize(
    off_t* size)
{
    if(!init_ok)
    {
        emmc_init();
    }

    *size = emmc_capacity();

    return OS_SUCCESS;
}

OS_Error_t
storage_rpc_getBlockSize(
    size_t* blockSize    
)
{
    *blockSize = emmc_block_size();
    return OS_SUCCESS;
}

OS_Error_t
__attribute__((__nonnull__))
storage_rpc_getState(
    uint32_t* flags)
{
    if(init_ok){
        *flags = 0;
    } else
    {
        *flags = 1;
    }
    
    return OS_SUCCESS;
}
