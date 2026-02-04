/*
MIT License

Copyright (c) 2021 DarkElvenAngel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <errno.h>
#include <time.h>
#include "shm_client.h"

// Default timeout for IPC operations (5 seconds)
#define IPC_TIMEOUT_MS 5000

/**
 * Send Request and wait for reply
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Send_Request(ArgonMem* ar_ptr)
{
    if (ar_ptr == NULL) return ENOTCONN;
    if (ar_ptr->daemon_pid != 0)
    {
      return kill(ar_ptr->daemon_pid, 1);
    }
    uint8_t last_state = 0;
    if (ar_ptr->memory->status != REQ_WAIT)
    {
        return EBUSY;
    }
    ar_ptr->memory->status = REQ_RDY;
    
    // Add timeout to prevent indefinite hangs
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (ar_ptr->memory->status != REQ_WAIT)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 + 
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms > IPC_TIMEOUT_MS)
        {
            return ETIMEDOUT;
        }
        
        if (last_state != ar_ptr->memory->status)
        {
            last_state = ar_ptr->memory->status;
            if (ar_ptr->memory->status == REQ_ERR)
            {
                return -1;
            }
        }
        msync(ar_ptr->memory, SHM_SIZE, MS_SYNC);
        usleep(1000);  // Small delay to reduce CPU usage
    }
    ar_ptr->memory->status = REQ_CLR;
    return 0;
}

/**
 * Send Reset Request and wait for reply
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Send_Reset(ArgonMem* ar_ptr)
{
    if (ar_ptr == NULL) return ENOTCONN;
    if (ar_ptr->daemon_pid != 0)
    {
      return kill(ar_ptr->daemon_pid, 1);
    }
    uint8_t last_state = 0;
    if (ar_ptr->memory->status != REQ_WAIT)
    {
        return EBUSY;
    }
    ar_ptr->memory->status = REQ_CLR;
    
    // Add timeout to prevent indefinite hangs
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (ar_ptr->memory->status != REQ_WAIT)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 + 
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms > IPC_TIMEOUT_MS)
        {
            return ETIMEDOUT;
        }
        
        if (last_state != ar_ptr->memory->status)
        {
            last_state = ar_ptr->memory->status;
            if (ar_ptr->memory->status == REQ_ERR)
            {
                return -1;
            }
        }
        msync(ar_ptr->memory, SHM_SIZE, MS_SYNC);
        usleep(1000);  // Small delay to reduce CPU usage
    }
    return 0;
}
/**
 * Send Reset Statitsics Request and wait for reply
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Send_Reset_Statistics(ArgonMem* ar_ptr)
{
    if (ar_ptr == NULL) return ENOTCONN;
    if (ar_ptr->daemon_pid != 0)
    {
      return kill(ar_ptr->daemon_pid, 1);
    }
    uint8_t last_state = 0;
    if (ar_ptr->memory->status != REQ_WAIT)
    {
        return EBUSY;
    }
    ar_ptr->memory->req_flags |= REQ_FLAG_STAT;
    ar_ptr->memory->status = REQ_CLR;
    
    // Add timeout to prevent indefinite hangs
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (ar_ptr->memory->status != REQ_WAIT)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 + 
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms > IPC_TIMEOUT_MS)
        {
            return ETIMEDOUT;
        }
        
        if (last_state != ar_ptr->memory->status)
        {
            last_state = ar_ptr->memory->status;
            if (ar_ptr->memory->status == REQ_ERR)
            {
                return EAGAIN;
            }
        }
        msync(ar_ptr->memory, SHM_SIZE, MS_SYNC);
        usleep(1000);  // Small delay to reduce CPU usage
    }
    return 0;
}

// Keep old name for backward compatibility
int Send_Reset_Statitsics(ArgonMem* ar_ptr)
{
    return Send_Reset_Statistics(ar_ptr);
}

/**
 * Get data inline not sure how time will work yet
 *
 * \return data in requested format
 *
 */
//  struct DTBO_Config Get_Config(ArgonMem* ar_ptr) return (struct DTBO_Config){ 0 };
//  struct SHM_DAEMON_STATS Get_Statistics(ArgonMem* ar_ptr)
//  uint8_t Get_Current_Temperature(ArgonMem* ar_ptr)
//  uint8_t Get_Current_FanSpeed(ArgonMem* ar_ptr)

/**
 * Get schedule structure from daemon
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Get_Config(ArgonMem* ar_ptr, struct DTBO_Config *config)
{
    if (ar_ptr == NULL)
    {
        return ENOTCONN;
    }
    if (config == NULL)
    {
        return ECANCELED;
    }
    *config = ar_ptr->memory->config;
    return 0;
}

/**
 * Get Statistics structure from daemon
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Get_Statistics(ArgonMem* ar_ptr, struct SHM_DAEMON_STATS *stat)
{

    if (ar_ptr == NULL)
    {
        return ENOTCONN;
    }
    if (stat == NULL)
    {
        return ECANCELED;
    }
    *stat = ar_ptr->memory->stat;
    return 0;
}

/**
 * Get Current CPU Temperature from daemon
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param temperature pointer to save temperature to
 * \return 0 on success
 */
int Get_Current_Temperature(ArgonMem* ar_ptr, uint8_t *temperature)
{
    if (ar_ptr == NULL)
    {
        return ENOTCONN;
    }
    if (temperature == NULL)
    {
        return ECANCELED;
    }
    *temperature = ar_ptr->memory->temperature;
    return 0;
}

/**
 * Get fan Speed from Daemon
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param speed pointer to save speed to
 * \return 0 on success
 */
int Get_Current_FanSpeed(ArgonMem* ar_ptr, uint8_t *speed)
{
    if (ar_ptr == NULL)
    {
        return ENOTCONN;
    }
    if (speed == NULL)
    {
        return ECANCELED;
    }
    *speed = ar_ptr->memory->fanspeed;
    return 0;
}

/**
 * Get new ArgonMem
 *
 * \return pointer to ArgonMem
 */
ArgonMem* New_ArgonMem()
{
    ArgonMem *r = (ArgonMem*)malloc(sizeof(ArgonMem));
    if (r != NULL)
    {
        r->shm_fd = 0;
        r->memory = NULL;
        r->daemon_pid = 0;
    }
    return r;
}

/**
 * Open the Argon one memory interface
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Open_ArgonMem(ArgonMem* ar_ptr)
{
    if (ar_ptr == NULL){
        return ENOMEM;
    }
    FILE* file = fopen (LOCK_FILE, "r");
    if (file == NULL)
    {
        if (errno == 2)
        {
          return ENOTCONN;
        }
        if (errno != 13)
        {
         return errno;
        }
    } else {
      if (fscanf (file, "%d", &ar_ptr->daemon_pid) != 1)
      {
          fclose(file);
          return EINVAL;
      }
      fclose (file);
      if (kill(ar_ptr->daemon_pid, 0) != 0)
      {
          return ENODEV;
      }
    }
    ar_ptr->shm_fd = shm_open(SHM_FILE, O_RDWR, 0660);
    if (ar_ptr->shm_fd == -1) return errno;
    if (ftruncate(ar_ptr->shm_fd, SHM_SIZE) == -1) {
        int saved_errno = errno;
        close(ar_ptr->shm_fd);
        ar_ptr->shm_fd = 0;
        return saved_errno;
    }
    ar_ptr->memory = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, ar_ptr->shm_fd, 0);
    if (ar_ptr->memory == MAP_FAILED)
    {
        int saved_errno = errno;
        fprintf(stderr, "ERROR:  Shared memory map error\n");
        close(ar_ptr->shm_fd);
        ar_ptr->shm_fd = 0;
        return saved_errno;
    }
    return 0;
}

/**
 * Close Argon one memory interface
 *
 * \param ar_ptr_ref Pointer to pointer to ArgonMem Struct (sets caller's pointer to NULL)
 */
void Close_ArgonMem(ArgonMem** ar_ptr_ref)
{
    if (ar_ptr_ref == NULL || *ar_ptr_ref == NULL) return;
    ArgonMem* ar_ptr = *ar_ptr_ref;
    munmap(ar_ptr->memory, SHM_SIZE);
    close(ar_ptr->shm_fd);
    ar_ptr->shm_fd = 0;
    ar_ptr->memory = NULL;
    ar_ptr->daemon_pid = 0;
    free(ar_ptr);
    *ar_ptr_ref = NULL;
}

/**
 * Setup fan mode change for request
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \return 0 on success
 */
int Set_FanMode(ArgonMem* ar_ptr, ArgonModes mode_select)
{
    if (ar_ptr == NULL) return ENOMEM;
    ar_ptr->memory->msg_app[0].fanmode = (uint8_t)mode_select;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_MODE;
    return 0;
}

/**
 * Set request fan speed  for manual or cooldown
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param speed fan speed request
 * \return 0 on success
 */
int Set_FanSpeed(ArgonMem* ar_ptr, uint8_t speed)
{
    if (ar_ptr == NULL) return ENOMEM;
    ar_ptr->memory->msg_app[0].fanspeed_Override = (uint8_t)speed;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_MODE;
    return 0;
}

/**
 * Set Request Target Target temperature
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param temperature Set target temperature for cooldown
 * \return 0 on success
 */
int Set_TargetTemperature(ArgonMem* ar_ptr, uint8_t temperature)
{
    if (ar_ptr == NULL) return ENOMEM;
    ar_ptr->memory->msg_app[0].temperature_target = (uint8_t)temperature;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_MODE;
    return 0;
}

/**
 * Setup fan request to change the fan schedule
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param Schedule data you want changed.
 * \return 0 on success
 */
int Set_Schedule(ArgonMem* ar_ptr, struct DTBO_Config Schedule)
{
    if (ar_ptr == NULL) return ENOMEM;
    ar_ptr->memory->msg_app[0].Schedules = Schedule;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_CONF;
    return 0;
}

/**
 * Setup fan mode to Cooldown mode change for request
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param temperature At what temperature should cooldown end
 * \param speed At what speed should te fan be set to during cooldown
 * \return 0 on success
 */
int Set_CoolDown(ArgonMem* ar_ptr, uint8_t temperature, uint8_t speed)
{
    if (ar_ptr == NULL) return ENOMEM;
    ar_ptr->memory->msg_app[0].temperature_target = (uint8_t)temperature;
    ar_ptr->memory->msg_app[0].fanspeed_Override = (uint8_t)speed;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_MODE;
    return 0;
}

/**
 * Setup fan mode to Manual and request fan speed change for request
 *
 * \param ar_ptr Pointer to ArgonMem Struct
 * \param speed what speed do you want the fan at
 * \return 0 on success
 */
int Set_ManualFan(ArgonMem* ar_ptr, uint8_t speed)
{
    if (ar_ptr == NULL) return ENOMEM;
    Set_FanMode(ar_ptr, AR_MODE_MAN);
    ar_ptr->memory->msg_app[0].fanspeed_Override = (uint8_t)speed;
    ar_ptr->memory->msg_app[0].req_flags |= REQ_FLAG_MODE;
    return 0;
}
