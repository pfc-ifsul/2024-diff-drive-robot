/*
 * Arquivo de configuração do FreeRTOS.
 *
 * Este arquivo define nomes utilizados para a configuração das tarefas do FreeRTOS.
 */

#ifndef CONFIG_FREERTOS_H
#define CONFIG_FREERTOS_H

#define TASK_CORE_0             0     /* Núcleo 0 da esp32 */
#define TASK_CORE_1             1     /* Núcleo 1 da esp32 */
#define TASK_MEMORY_SIZE_1KB    1024  /* Tamanho de memória destinada a tarefa */
#define TASK_MEMORY_SIZE_2KB    2048  /* Tamanho de memória destinada a tarefa */
#define TASK_MEMORY_SIZE_4KB    4096  /* Tamanho de memória destinada a tarefa */
#define TASK_MEMORY_SIZE_8KB    8192  /* Tamanho de memória destinada a tarefa */
#define TASK_PRIORITY_ONE       1     /* Prioridade de tarefa 1 */
#define TASK_PRIORITY_TWO       2     /* Prioridade de tarefa 2 */
#define TASK_PRIORITY_THREE     3     /* Prioridade de tarefa 2 */

#endif /* CONFIG_FREERTOS_H */