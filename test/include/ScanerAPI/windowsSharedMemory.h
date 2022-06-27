#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#pragma comment(lib, "user32.lib")


#define BUF_SIZE 4096
// All the following methods will create/access to the same Named shared memory
TCHAR shmName[] = TEXT("Local\\SCANeR_SK_LAUNCHER_SHM");

int createSharedMemory(HANDLE& handleMapFile)
{
	handleMapFile = CreateFileMapping(INVALID_HANDLE_VALUE,    // use paging file
										NULL,                    // default security
										PAGE_READWRITE,          // read/write access
										0,                       // maximum object size (high-order DWORD)
										BUF_SIZE,                // maximum object size (low-order DWORD)
										shmName);                 // name of mapping object

	if (handleMapFile == NULL)
	{
		_tprintf(TEXT("Could not create file mapping object (%d).\n"), GetLastError());
		return 1;
	}
	return 0;
}

void deleteSharedMemory(HANDLE handleMapFile)
{
	CloseHandle(handleMapFile);
}

int readSharedMemory(std::string& memoryContent)
{
	HANDLE handleMapFile;
	LPCTSTR pBuf;

	handleMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read/write access
									FALSE,                 // do not inherit the name
									shmName);              // name of mapping object

	if (handleMapFile == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"), GetLastError());
		return 1;
	}

	pBuf = (LPTSTR) MapViewOfFile(handleMapFile, // handle to map object
									FILE_MAP_ALL_ACCESS,  // read/write permission
									0,
									0,
									BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError());
		CloseHandle(handleMapFile);
		return 1;
	}

	memoryContent = pBuf;

	UnmapViewOfFile(pBuf);
	CloseHandle(handleMapFile);

	return 0;
}

int writeToSharedMemory(const std::string& textToCopy)
{
	HANDLE handleMapFile;
	LPCTSTR pBuf;

	handleMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read/write access
									FALSE,                 // do not inherit the name
									shmName);              // name of mapping object

	if (handleMapFile == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"), GetLastError());
		return 1;
	}

	pBuf = (LPTSTR) MapViewOfFile(handleMapFile, // handle to map object
									FILE_MAP_ALL_ACCESS,  // read/write permission
									0,
									0,
									BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError());
		CloseHandle(handleMapFile);
		return 1;
	}

	CopyMemory((PVOID)pBuf, textToCopy.c_str(), ((textToCopy.length()+1) * sizeof(TCHAR)));

	UnmapViewOfFile(pBuf);
	CloseHandle(handleMapFile);

	return 0;
}

int clearSharedmemory(HANDLE handleMapFile)
{
	LPCTSTR pBuf;

	handleMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read/write access
									FALSE,                 // do not inherit the name
									shmName);              // name of mapping object

	if (handleMapFile == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"), GetLastError());
		return 1;
	}

	pBuf = (LPTSTR) MapViewOfFile(handleMapFile, // handle to map object
									FILE_MAP_ALL_ACCESS,  // read/write permission
									0,
									0,
									BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError());
		CloseHandle(handleMapFile);
		return 1;
	}

	ZeroMemory((PVOID)pBuf, BUF_SIZE);

	UnmapViewOfFile(pBuf);
	CloseHandle(handleMapFile);

	return 0;
}