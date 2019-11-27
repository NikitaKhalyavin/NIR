/***************************************************************************************************
                              Это файл, который затыкает собой щели.
 
 В нем лежит atexit, assert_failed, обвязка для printf'a и всякая веселуха
 
***************************************************************************************************/


/**************************************************************************************************
 Это мы так запихиваем по фиксированному адресу время компиляции и хеш коммита
 Строчка с define COMMIT_DATA генерируется скриптом update_commit_hash.bat, который нужно
 вызывать до билда. Чтобы от изменения этого файла не изменялась рабочая копия, после билда нужно
 вызывать скрипт remove_commit_hash.bat.
 
 Для этого в меню project->options->user нужно выбрать соответствующие скрипты (по относительному пути)
 в пунктах Before build/rebuild и After build/rebuild.
 
 Для использования этого скрипта нужен bash от гита. Чтобы помочь скрипту его найти, создайте
 переменную окружения GIT_BASH_PATH (95% что она будет равна "C:\Program Files (x86)\Git\usr\bin\" 
 
 Строчка текста с инфой будет лежать в памяти. Если вы хотите положить ее по какому-то определнному 
 адресу, создайте дефайн RETARGET_BUILD_INFO_ADDRESS, равный этому адресу.

 Эта функциональность включена по-умолчанию. 
 Для отключеная - создайте дефайн RETARGET_DISABLE_BUILD_INFO.
 
 Все дефайны можно класть в project_config.h.
 

**************************************************************************************************/

#include "stm32f4xx_hal.h"

extern "C" 
{

    /**************************************************************************************************
    Описание:  Эта функция вызывается в конструкторе каждого объекта со статическим временем жизни,
               чтобы "зарегистрировать" в динамически выделяемом списке. После выхода из main, вызовется
               _sys_exit, который для всех этих объектов вызывает деструкторы. Поскольку у нас main 
               никогда не завершается, все это абсолютно бессмысленно и только зря память жрет.
               ARM официально разрешает переопределить эту функцию.
    Аргументы: Какие-то есть, но игнорируются.
    Возврат:   Положительное число означает, что все хорошо.
    Замечания: Гуглите __aeabi_atexit и читайте официальную доку от ARM, если хотите узнать больше.
    **************************************************************************************************/
    int __aeabi_atexit(void)
    {
        return 1;
    }
}

/***************************************************************************************************
  Следующий блок функций делает возможным использования printf в симуляторе keil'a.
  Чтобы выключить - объявите символ UMBA_DONT_USE_RETARGET.
  
  Данная реализация является минимальной и не будет работать вне симулятора.
  
***************************************************************************************************/
#ifndef UMBA_DONT_USE_RETARGET 

// если компилятор - armcc или keil-clang
#if __CC_ARM || ( (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) )

    #if ( (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) )

        asm(".global __use_no_semihosting_swi\n");
        
    #elif __CC_ARM
    
        #pragma import(__use_no_semihosting_swi)
        
        namespace std { struct __FILE { int handle;} ; }
    
    #endif
    
    #include <stdio.h>
    #include <rt_sys.h>
    #include <rt_misc.h>


    std::FILE std::__stdout;
    std::FILE std::__stdin;
    std::FILE std::__stderr;
    
    extern "C"
    { 
        int fputc(int c, FILE *f)
        {
            (void)f;
            return ITM_SendChar(c);
        }

        int fgetc(FILE *f)
        {
            char ch = 0;

            return((int)ch);
        }

        int ferror(FILE *f)
        {
            /* Your implementation of ferror */
            return EOF;
        }

        void _ttywrch(int c)
        {
            ITM_SendChar(c);
        }
        
        char *_sys_command_string(char *cmd, int len)
        {
            return NULL;
        }
        
        // вызывается после main
        void _sys_exit(int return_code) 
        {
            while(1);
        }        
    }
#endif

#endif