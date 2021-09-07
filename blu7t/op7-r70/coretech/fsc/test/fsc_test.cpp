#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <pthread.h>

#include <android-base/file.h>
#include <android-base/macros.h>
#include <android-base/stringprintf.h>
#include <gtest/gtest.h>

#define TARGET_ALLOW_LIST "/data/local/tmp"
#define FSC_ENABLE "/sys/module/fsc/parameters/enable"
#define ADD_ALLOW_LIST_ENTRY "/sys/module/fsc/parameters/allow_list_add"
#define DEL_ALLOW_LIST_ENTRY "/sys/module/fsc/parameters/allow_list_del"
#define ALLOW_LIST_DUMP "/proc/fsc_allow_list"

#define EMPTY_FILE "/data/local/tmp/fsc_test_file"
#define EMPTY_FILE_ALIAS_0 "/data/local/tmp/fsc_test_file"
#define EMPTY_FILE_ALIAS_1 "fsc_test_file"
#define EMPTY_FILE_ALIAS_2 "./fsc_test_file"
#define EMPTY_FILE_ALIAS_3 "../tmp/./././fsc_test_file"
#define EMPTY_FILE_ALIAS_4 ".././././//./tmp/fsc_test_file"
#define EMPTY_FILE_ALIAS_5 ".././//tmp/././//./fsc_test_file"

#define PREPARE_FILE "/data/local/tmp/fsc_prepare_file"


static void fsc_test_init(void) {
	/* reset fsc first */
	android::base::WriteStringToFile("0", FSC_ENABLE);
	android::base::WriteStringToFile("1", FSC_ENABLE);
	android::base::WriteStringToFile(TARGET_ALLOW_LIST, ADD_ALLOW_LIST_ENTRY);
}

static void fsc_test_deinit(void) {
	android::base::WriteStringToFile(TARGET_ALLOW_LIST, DEL_ALLOW_LIST_ENTRY);
}

static bool fsc_check_allow_list(void) {
	std::ifstream infile(ALLOW_LIST_DUMP);
	std::string path;
	while (std::getline(infile, path)) {
		int coma = path.find(",");
		path = path.substr(0, coma);
		if (path == TARGET_ALLOW_LIST)
			return true;
	}
	return false;
}

static int fsc_check_file_exist(const char *name) {
	struct stat sb;
	return stat(name, &sb);
}

static int fsc_create_file(const char *name, int flags, mode_t mode) {
	int fd = open(name, flags, mode);
	if (fd) EXPECT_EQ(close(fd), 0);
	return fd;
}

TEST(fsc, add_allow_list) {
	int cnt = 1000;
	while (--cnt) {
		fsc_test_init();
		ASSERT_TRUE(fsc_check_allow_list());
		fsc_test_deinit();
	}
}

/*
 * testing coverage:
 * open related
 */
#define GEN_TEST_OPEN_ALIAS(num) \
TEST(fsc, open_alias_##num) { \
	const char* target = EMPTY_FILE_ALIAS_##num; \
	fsc_test_init(); \
	ASSERT_TRUE(fsc_check_allow_list()); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), -1); \
	EXPECT_GE(fsc_create_file(target, O_CREAT, S_IRUSR|S_IWUSR), 0); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(EMPTY_FILE), 0); \
	fsc_test_deinit(); \
}

GEN_TEST_OPEN_ALIAS(0);
GEN_TEST_OPEN_ALIAS(1);
GEN_TEST_OPEN_ALIAS(2);
GEN_TEST_OPEN_ALIAS(3);
GEN_TEST_OPEN_ALIAS(4);
GEN_TEST_OPEN_ALIAS(5);

/*
 * testing coverage:
 * mv rename
 */
#define GEN_TEST_RENAME_ALIAS(num) \
TEST(fsc, rename_alias_##num) { \
	const char* from  = PREPARE_FILE; \
	const char* to = EMPTY_FILE_ALIAS_##num; \
	fsc_test_init(); \
	ASSERT_TRUE(fsc_check_allow_list()); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), -1); \
	EXPECT_GE(fsc_create_file(from, O_CREAT, S_IRUSR|S_IWUSR), 0); \
	EXPECT_EQ(rename(from, to), 0); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(EMPTY_FILE), 0); \
	fsc_test_deinit(); \
}

GEN_TEST_RENAME_ALIAS(0)
GEN_TEST_RENAME_ALIAS(1)
GEN_TEST_RENAME_ALIAS(2)
GEN_TEST_RENAME_ALIAS(3)
GEN_TEST_RENAME_ALIAS(4)
GEN_TEST_RENAME_ALIAS(5)

/*
 * testing coverage:
 * link symlink
 */
#define GEN_TEST_LINK_ALIAS(num) \
TEST(fsc, link_##num) { \
	const char* from  = PREPARE_FILE; \
	const char* to = EMPTY_FILE_ALIAS_##num; \
	fsc_test_init(); \
	ASSERT_TRUE(fsc_check_allow_list()); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), -1); \
	EXPECT_GE(fsc_create_file(from, O_CREAT, S_IRUSR|S_IWUSR), 0); \
	EXPECT_EQ(link(from, to), 0); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(from), 0); \
	fsc_test_deinit(); \
}

GEN_TEST_LINK_ALIAS(0)
GEN_TEST_LINK_ALIAS(1)
GEN_TEST_LINK_ALIAS(2)
GEN_TEST_LINK_ALIAS(3)
GEN_TEST_LINK_ALIAS(4)
GEN_TEST_LINK_ALIAS(5)

#define GEN_TEST_SYMLINK_ALIAS(num) \
TEST(fsc, symlink_##num) { \
	const char* from  = PREPARE_FILE; \
	const char* to = EMPTY_FILE_ALIAS_##num; \
	fsc_test_init(); \
	ASSERT_TRUE(fsc_check_allow_list()); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), -1); \
	EXPECT_GE(fsc_create_file(from, O_CREAT, S_IRUSR|S_IWUSR), 0); \
	EXPECT_EQ(symlink(from, to), 0); \
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(EMPTY_FILE), 0); \
	EXPECT_EQ(unlink(from), 0); \
	fsc_test_deinit(); \
}

GEN_TEST_SYMLINK_ALIAS(0)
GEN_TEST_SYMLINK_ALIAS(1)
GEN_TEST_SYMLINK_ALIAS(2)
GEN_TEST_SYMLINK_ALIAS(3)
GEN_TEST_SYMLINK_ALIAS(4)
GEN_TEST_SYMLINK_ALIAS(5)

/*
 * testing for race
 */
static void *thread_stat(void *arg __unused) {
	struct stat sb;
	while (stat(EMPTY_FILE, &sb));
	return NULL;
}

static void *thread_open(void *arg __unused) {
	int fd = -1;
	fd = open(EMPTY_FILE, O_CREAT, 0644);
	if (fd) close(fd);
	return NULL;
}

#define TEST_ROUNDS (100000)
#define THREAD_MAX (10)
TEST(fsc, race) {
	int ceil, i = 0;
	int test = 0, test_ceil = TEST_ROUNDS;
	pthread_t stat_tids[THREAD_MAX];
	pthread_t open_tid;

	fsc_test_init();
	ASSERT_TRUE(fsc_check_allow_list());
	EXPECT_EQ(fsc_check_file_exist(EMPTY_FILE), -1);

	GTEST_LOG_(INFO) << "RACE TEST: this test will take around 3 minutes to complete.";
	while (test++ != test_ceil) {
		srand(time(NULL));
		ceil = rand() % THREAD_MAX;

		for (i = 0; i <= ceil; ++i)
			pthread_create(&stat_tids[i], NULL, &thread_stat, NULL);

		pthread_create(&open_tid, NULL, &thread_open, NULL);

		for (i = 0; i <= ceil; ++i)
			pthread_join(stat_tids[i], NULL);

		pthread_join(open_tid, NULL);

		EXPECT_EQ(unlink(EMPTY_FILE), 0);
		//GTEST_LOG_(INFO) << "RACE TEST: (" << test << "/" << test_ceil << ") pass";
	}
	fsc_test_deinit();
}
