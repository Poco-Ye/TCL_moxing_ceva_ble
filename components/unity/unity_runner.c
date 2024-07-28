// Copyright 2016-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include "unity.h"
#include "unity_config.h"
#include "unity_test_runner.h"
#include "log.h"


/* similar to UNITY_PRINT_EOL */
#define UNITY_PRINT_TAB() UNITY_OUTPUT_CHAR('\t')

// Pointers to the head and tail of linked list of test description structs:
//static test_desc_t *s_unity_tests_first = NULL;
//static test_desc_t *s_unity_tests_last = NULL;

test_menu_t *s_unity_tests_first = NULL;
test_menu_t *s_unity_tests_last = NULL;

extern bool unity_pushdata(void);


void unity_testcase_register(test_desc_t *desc)
{
    test_menu_t *menu = NULL;
	
    if (!s_unity_tests_first) {		
		menu = (test_menu_t *) malloc(sizeof(test_menu_t));
		menu->name=desc->menuname;
		menu->item = NULL;
		menu->next =NULL;
        s_unity_tests_first = menu;
        s_unity_tests_last = menu;
    } else {
		for (menu = s_unity_tests_first; menu != NULL; menu = menu->next) {
			if (strcmp(menu->name, desc->menuname) == 0) {
				break;
			}
		}
		if (menu ==NULL)
		{
   			test_menu_t *temp = NULL;
			menu = (test_menu_t *) malloc(sizeof(test_menu_t));
			menu->name=desc->menuname;
			menu->item = NULL;
			temp = s_unity_tests_first;
			s_unity_tests_first = menu;
			menu->next =temp;
		}	
		
	
    }
	
	if (menu->item == NULL) {
		menu->item = desc;
		menu->item->next =NULL;
	} else {
		test_desc_t *temp = menu->item;
		menu->item = desc;
		menu->item->next = temp;
	}	
}


/* print the multiple function case name and its sub-menu
 * e.g:
 * (1) spi master/slave case
 *       (1)master case
 *       (2)slave case
 * */

/*pyxue
static void print_multiple_function_test_menu(const test_desc_t *test_ms)
{
    UnityPrint(test_ms->name);
    UNITY_PRINT_EOL();
    for (int i = 0; i < test_ms->test_fn_count; i++) {
        UNITY_PRINT_TAB();
        UnityPrint("(");
        UnityPrintNumberUnsigned(i + 1);
        UnityPrint(")");
        UNITY_PRINT_TAB();
        UnityPrint("\"");
        UnityPrint(test_ms->test_fn_name[i]);
        UnityPrint("\"");
        UNITY_PRINT_EOL();
    }
}
*/


/*
 * This function looks like UnityDefaultTestRun function only without UNITY_CLR_DETAILS.
 * void UnityDefaultTestRun(UnityTestFunction Func, const char* FuncName, const int FuncLineNum)
 * was moved from `components/unity/unity/src/unity.c` to here.
*/
static void unity_default_test_run(UnityTestFunction Func, const char* FuncName, const int FuncLineNum)
{
    Unity.CurrentTestName = FuncName;
    Unity.CurrentTestLineNumber = (UNITY_LINE_TYPE)FuncLineNum;
    Unity.NumberOfTests++;
    if (TEST_PROTECT())
    {
        setUp();
        Func();
    }
    if (TEST_PROTECT())
    {
        tearDown();
    }
    UnityConcludeTest();
}

/*pyxue
static int multiple_function_option(const test_desc_t *test_ms)
{
    int selection;
    char cmdline[256] = {0};

    print_multiple_function_test_menu(test_ms);
    while (strlen(cmdline) == 0) {
        unity_gets(cmdline, sizeof(cmdline));
        if (strlen(cmdline) == 0) {
            // if input was newline, print a new menu
            print_multiple_function_test_menu(test_ms);
        }
    }
    selection = atoi((const char *) cmdline) - 1;
    if (selection >= 0 && selection < test_ms->test_fn_count) {
        unity_default_test_run(test_ms->fn[selection], test_ms->name, test_ms->line);
    } else {
        UnityPrint("Invalid selection, your should input number 1-");
        UnityPrintNumber(test_ms->test_fn_count);
        UNITY_PRINT_EOL();
    }
    return selection;
}*/

static void unity_run_single_test(const test_desc_t *test)
{
    UnityPrint("Running ");
    UnityPrint(test->name);
    UnityPrint("...");
    UNITY_PRINT_EOL();
    // Unit test runner expects to see test name before the test starts
    UNITY_OUTPUT_FLUSH();

    Unity.TestFile = test->file;
    Unity.CurrentDetail1 = test->desc;
    //bool reset_after_test = strstr(Unity.CurrentDetail1, "[leaks") != NULL;
    //bool multi_device = strstr(Unity.CurrentDetail1, "[multi_device]") != NULL;
//    if (test->test_fn_count == 1) {
        unity_default_test_run(test->fn[0], test->name, test->line);
  //  } else {
  //      int selection = multiple_function_option(test);
  //      if (reset_after_test && multi_device == false) {
  //          if (selection != (test->test_fn_count - 1)) {
  //              // to do a reset for all stages except the last stage.
  //              esp_restart();
  //          }
  //      }
   // }

    // pyxue if (reset_after_test) {
        // print a result of test before to do reset for the last stage.
       // UNITY_END();
       // UnityPrint("Enter next test, or 'enter' to see menu");
       // UNITY_PRINT_EOL();
       // UNITY_OUTPUT_FLUSH();
    //    esp_restart();
  //  }
}

static void unity_run_single_test_by_index(int index)
{
    const test_menu_t *menu;
    const test_desc_t *test;
    for (menu = s_unity_tests_first; menu != NULL; menu = menu->next) {
	    for (test = menu->item; test != NULL && index != 0; test = test->next, --index) {
	        ;
	    }
		if ((test ==NULL) && (menu !=NULL)) {
			continue;
		}
		if (index==0) {break;}
    }
	
    if (test != NULL) {
        unity_run_single_test(test);
    }
}

static void unity_run_single_test_by_index_parse(const char *filter, int index_max)
{
    int test_index = strtol(filter, NULL, 10);
    if (test_index >= 1 && test_index <= index_max) {
        UNITY_EXEC_TIME_START();
        unity_run_single_test_by_index(test_index - 1);
        UNITY_EXEC_TIME_STOP();
        UnityPrint("Test ran in ");
        UnityPrintNumberUnsigned(UNITY_EXEC_TIME_MS());
        UnityPrint("ms");
        UNITY_PRINT_EOL();
        UNITY_OUTPUT_FLUSH();
    }
}

void unity_run_test_by_name(const char *name)
{
    const test_menu_t *menu;
    for (menu = s_unity_tests_first; menu != NULL; menu = menu->next) {
	    for (const test_desc_t *test = menu->item; test != NULL; test = test->next) {
	        if (strcmp(test->name, name) == 0) {
	            unity_run_single_test(test);
	        }
	    }
    }
}

void unity_run_all_tests(void)
{
    const test_menu_t *menu;
    for (menu = s_unity_tests_first; menu != NULL; menu = menu->next) {
	    for (const test_desc_t *test = menu->item; test != NULL; test = test->next) {
	        unity_run_single_test(test);
	    }
    }
}

void unity_run_all_submenu_tests(int iMenuIdx)
{
    const test_menu_t *menu;
	int iCount =iMenuIdx-1;
		
    for (menu = s_unity_tests_first; (menu != NULL) && (iCount!=0); menu = menu->next, iCount--) {
		;	
    }
	if ((menu != NULL) && (iCount==0))
	{
	    for (const test_desc_t *test = menu->item; test != NULL; test = test->next) {
	        unity_run_single_test(test);
	    }
	}
}


void unity_query_status(void)
{
    UnityPrint("CurrentStatus:");
	UnityEnd();

}

void unity_run_tests_by_tag(const char *tag, bool invert)
{
    const test_menu_t *menu;
	
    UnityPrint("Running tests ");
    if (invert) {
        UnityPrint("NOT ");
    }
    UnityPrint("matching '");
    UnityPrint(tag);
    UnityPrint("'...");
    UNITY_PRINT_EOL();

    for (menu = s_unity_tests_first; menu != NULL; menu = menu->next) {
	    for (const test_desc_t *test = menu->item; test != NULL; test = test->next) {

	        if ((strstr(test->desc, tag) != NULL) == !invert) {
	            unity_run_single_test(test);
	        }
	    }
    }
}

static void removespaceretuen(char *str)
{
    int temp;
	int length = 0;
    char buffer[256];

    strcpy(buffer, str);

	for(temp = 0; temp < strlen(buffer); temp ++)
	{
        if((buffer[temp] != 0xa) && (buffer[temp] != 0xd)  && (buffer[temp] != ' ')	)
		{
		    str[length] = buffer[temp];
            length ++;
        }
	}
	str[length] = 0;
}

static int print_test_menu_ex(int iMenuIdx)
{
    int test_menu_counter = 0;
    int test_menu_items_counter = 0;
    UNITY_PRINT_EOL();
    UNITY_PRINT_EOL();
	UnityPrint("User Guide: <Num>:Enter submenu or run one testcase;  <Space>:Show currenet menu;  a:Show all testcase;  b:Back to main menu;  *:run all testcase in current menu;");
 	UNITY_PRINT_EOL();
    UnityPrint("Here's the test menu, pick your combo:");
    UNITY_PRINT_EOL();
    for (const test_menu_t *menu = s_unity_tests_first; menu != NULL; menu = menu->next) 
	{
		test_menu_counter++;
		if (test_menu_counter ==iMenuIdx)
		{
		    UnityPrint("Menu:");
		    UnityPrint(menu->name);
		    UNITY_PRINT_EOL();
		    for (const test_desc_t *test = menu->item;
		            test != NULL;
		            test = test->next, ++test_menu_items_counter) {
				UnityPrint(".");
				UnityPrint(".");
				UnityPrint(".");

		        UnityPrint("(");
		        UnityPrintNumber(test_menu_items_counter + 1);
		        UnityPrint(")");
		        UNITY_PRINT_TAB();
		        UnityPrint("\"");
		        UnityPrint(test->name);
		        UnityPrint("\" ");
		        UnityPrint(test->desc);
		        UNITY_PRINT_EOL();
		        if (test->test_fn_count > 1) {
		            for (int i = 0; i < test->test_fn_count; i++) {
		                UNITY_PRINT_TAB();
		                UnityPrint("(");
		                UnityPrintNumber(i + 1);
		                UnityPrint(")");
		                UNITY_PRINT_TAB();
		                UnityPrint("\"");
		                UnityPrint(test->test_fn_name[i]);
		                UnityPrint("\"");
		                UNITY_PRINT_EOL();
		            }
		        } 
		    }
		}
		else
		{
		    for (const test_desc_t *test = menu->item;
		            test != NULL;
		            test = test->next, ++test_menu_items_counter) {
		    }
		}
	}
    UNITY_PRINT_EOL();
    UnityPrint("Enter test for running."); /* unit_test.py needs it for finding the end of test menu */
    UNITY_PRINT_EOL();
    UNITY_OUTPUT_FLUSH();
    return test_menu_items_counter;
}

static int print_test_menu_item_forTest()
{
    int test_menu_items_counter = 0;
    UNITY_PRINT_EOL();
    UNITY_PRINT_EOL();
	
    for (const test_menu_t *menu = s_unity_tests_first; menu != NULL; menu = menu->next) 
	{
	    for (const test_desc_t *test = menu->item;
	            test != NULL;
	            test = test->next, test_menu_items_counter++) {

			        UnityPrintNumber(test_menu_items_counter );
			        UnityPrint(",");
			        UnityPrint(test->desc);
			        UnityPrint(",");
			       // UNITY_PRINT_TAB();
			        UnityPrint("\"");
			        UnityPrint(test->name);
			        UnityPrint("\"");
			        UnityPrint(";");
			        UNITY_PRINT_EOL();
	    }
	}
    UNITY_PRINT_EOL();
    UNITY_OUTPUT_FLUSH();
    return test_menu_items_counter;
}


static int print_test_menu_root(bool bShowAllItem)
{
    int test_menu_counter = 0;
    int test_menu_items_counter = 0;
    UNITY_PRINT_EOL();
    UNITY_PRINT_EOL();
	UnityPrint("User Guide: <Num>:Enter submenu or run one testcase;  <Space>:Show currenet menu;  a:Show all testcase;  b:Back to main menu;  *:run all testcase in current menu;");
 	UNITY_PRINT_EOL();
    UnityPrint("Here's the test menu list, pick your combo:");
    UNITY_PRINT_EOL();
    for (const test_menu_t *menu = s_unity_tests_first; menu != NULL; menu = menu->next) 
	{
	    UnityPrint("[");
	    UnityPrintNumber(++test_menu_counter);
	    UnityPrint("]:");
			
	    UnityPrint("""");
	    UnityPrint(menu->name);
	    UnityPrint("""");
		if (!bShowAllItem)
		{
			test_menu_items_counter=0;
		}
		else
		{
			UNITY_PRINT_EOL();
		}
	    for (const test_desc_t *test = menu->item;
	            test != NULL;
	            test = test->next, ++test_menu_items_counter) {
	            if (bShowAllItem)
	            {	
		            UnityPrint(".");
					UnityPrint(".");
					UnityPrint(".");

			        UnityPrint("(");
			        UnityPrintNumber(test_menu_items_counter + 1);
			        UnityPrint(")");
			        UNITY_PRINT_TAB();
			        UnityPrint("\"");
			        UnityPrint(test->name);
			        UnityPrint("\" ");
			        UnityPrint(test->desc);
			        UNITY_PRINT_EOL();
			        if (test->test_fn_count > 1) {
			            for (int i = 0; i < test->test_fn_count; i++) {
			                UNITY_PRINT_TAB();
			                UnityPrint("(");
			                UnityPrintNumber(i + 1);
			                UnityPrint(")");
			                UNITY_PRINT_TAB();
			                UnityPrint("\"");
			                UnityPrint(test->test_fn_name[i]);
			                UnityPrint("\"");
			                UNITY_PRINT_EOL();
			            }
			        } 
				}
	    }
		if (!bShowAllItem)
		{
		    UnityPrint("(");
		    UnityPrintNumber(test_menu_items_counter);
		    UnityPrint(")");
		    UNITY_PRINT_EOL();
		}
	}
    UNITY_PRINT_EOL();
    UnityPrint("Enter test menu for details."); /* unit_test.py needs it for finding the end of test menu */
    UNITY_PRINT_EOL();
    UNITY_OUTPUT_FLUSH();
    return test_menu_items_counter;
}

void unity_autotest_entry(const char * strcmd)
{
	if (strcmd[0]=='?')
	{
		unity_query_status();
	}
	else if (strcmd[0]=='-')
	{
		UNITY_BEGIN();
	}
	else if (strcmd[0]=='#')
	{
		print_test_menu_item_forTest();
	}
	else if ( (strcmd[0]=='T')&(strcmd[1]=='S')&(strcmd[2]=='T')&(strcmd[3]==':'))
	{
		unity_run_tests_by_tag(strcmd +4, false);
		UNITY_END();
	}
	return;
}


static int get_test_count(int * iMenuCount)
{
	int test_menucounter =0;
    int test_counter = 0;
    const test_menu_t *menu;
	const test_desc_t *test;
	
    for (menu = s_unity_tests_first; menu != NULL; menu = menu->next,test_menucounter++) {
		test = menu->item;
		if (test!=NULL)
		{
		    for (test = menu->item;
		            test != NULL;
		            test = test->next) {
		        ++test_counter;
		    }
			
		}
    }
	*iMenuCount = test_menucounter;
    return test_counter;
}

void unity_run_menu(void)
{
    bool commandready;
	bool invert = false;
	int idx = 0;
    char cmdline[256];
	int temp;
	bool bTestMode =false;
	char iMenuIdx =0;
	
    UNITY_PRINT_EOL();
    UNITY_PRINT_EOL();
    UnityPrint("Press ENTER to see the list of tests.");
    UNITY_PRINT_EOL();
	int test_menu_count=0;
    int test_count = get_test_count(&test_menu_count);


	print_test_menu_root(false);
    while (true) 
    {
    	memset (cmdline, 0,256);
		commandready = unity_pushdata();
		if(commandready == false)
            continue;
		
		commandready = false;
        unity_gets(cmdline, sizeof(cmdline));

		//for(temp = 0; temp < strlen(cmdline); temp++)
        //{
        //	MS_LOGI( MS_DRIVER, "cmdline[%d]:%d", temp, cmdline[temp]);    
        //}

				
		removespaceretuen(cmdline);
		//MS_LOGI( MS_DRIVER, "get command %s, length %d\r\n", cmdline, strlen(cmdline));

		if (strlen(cmdline) == 0) {
			/* if input was newline, print a new menu */
			if (iMenuIdx>0)
			{
				print_test_menu_ex(iMenuIdx);
			}
			else
			{
				print_test_menu_root(false);
			}
		}

        /*use '-' to show test history. Need to do it before UNITY_BEGIN cleanup history */

        if (cmdline[0] == '-') {
            UNITY_END();
            continue;
        }
		if (!bTestMode)
		{
        	UNITY_BEGIN();
		}

       /* bool invert = false;
        if (cmdline[idx] == '!') {
            invert = true;
            ++idx;
        }*/
        invert = false;

		if (bTestMode ==true)
		{
			if (cmdline[1]=='!'){
		        bTestMode =false;
			}
			else{
				unity_autotest_entry(cmdline + idx+1);
			}
		}
		else
		{
	        if (cmdline[idx] == '*') {
				if (iMenuIdx>0){
					unity_run_all_submenu_tests(iMenuIdx);
				}
				else {
	            	unity_run_all_tests();
				}
	        } else if (cmdline[idx] == '#') {
	        	bTestMode =true;
				unity_autotest_entry(cmdline + idx+1);
	        } else if (cmdline[idx] == 'a') {
				print_test_menu_root(true);
	        } else if (cmdline[idx] == 'b') {
	        	iMenuIdx=0;
				print_test_menu_root(false);
	        } else if (cmdline[idx] == '[') {
	            unity_run_tests_by_tag(cmdline + 1, invert);
	        } else if (cmdline[idx] == '"') {
	            char* end = strrchr(cmdline, '"');
	            if (end > &cmdline[idx]) {
	                *end = 0;
	                unity_run_test_by_name(cmdline + idx + 1);
	            }
	        } else if (isdigit((unsigned char)cmdline[idx])) {
	        	if (iMenuIdx>0)
	        	{
					unity_run_single_test_by_index_parse(cmdline + idx, test_count);
				}
				else
				{
					int itemp = strtol(cmdline + idx, NULL, 10);
					if ((itemp >0) && (iMenuIdx <=test_menu_count))
					{
						iMenuIdx =itemp;
						print_test_menu_ex(iMenuIdx);
					}
					else
					{
						print_test_menu_root(false);
					}
				}
	        }
		}

		if (iMenuIdx>0)
		{
				if (!bTestMode)
				{
		        	UNITY_END();
		        	UnityPrint("Enter next test, or 'enter' to see menu");
				}
				else
				{
					UNITY_END();
				}
		}

        UNITY_PRINT_EOL();
        UNITY_OUTPUT_FLUSH();
    }
}
