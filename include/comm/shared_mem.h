/* shared_mem.h
 *
 * Okke Hendriks - March 2013 - Tech United
 */
#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include <unistd.h>
#include <time.h>
#include <limits.h>
#include <semaphore.h>
#include <fcntl.h> /* For O_* constants */

#define NAME_MAX 16

//#define DEBUG_SHARED_MEM  // Enabled or disable printf's

typedef struct
{
    // Global semapohered shared mem variables
    key_t key;
    sem_t *sem;
    int shmid;
    void *segptr;
    int semid;
    int createdSegment;
} semShm_t, *semShm_p;

void *shmOpen(int key, int size, int *shmid, sem_t **sem, int *newSegment);
int shmRemove(int shmid, void *segptr);
int lockSemaphore(sem_t *sem, int blocking);
int shmWrite(void *object, int size, int shmid, void *segptr, sem_t *sem, int blocking, int lock);
int shmRead(void *object, int size, int shmid, void *segptr, sem_t *sem, int blocking, int unlock);
int shmReadWrite(void *object, int size, int shmid, void *segptr, sem_t *sem, int blocking, int (*f)(void *object));
int openSemForShm(int shmid, sem_t **sem);
int closeSemForShm(int shmid);
int getSemVal(sem_t *sem);

void tes();

#endif // SHARED_MEM_H