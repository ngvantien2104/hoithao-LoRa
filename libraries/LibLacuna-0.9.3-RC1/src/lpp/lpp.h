#ifndef __LPP_H__
#define __LPP_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


#define LPP_ERR_OK                  (0)
#define LPP_ERR_NO_SPACE            (-1)
#define LPP_ERR_NOT_INITIALIZED     (-2)
#define LPP_ERR_INVALID_DATA        (-3)
#define LPP_ERR_VERSION_MISMATCH    (-4)
#define LPP_ERR_DATASIZE            (-5)
#define LPP_ERR_NOT_FOUND           (-6)
#define LPP_ERR_RANGE               (-7)
#define LPP_ERR_TYPE                (-8)
#define LPP_ERR_UNEXPECTED_END      (-9)
#define LPP_ERR_INTERNAL            (-10)
#define LPP_ERR_ARITH               (-11)
#define LPP_ERR_NOT_AVAILABLE       (-12)
#define LPP_ERR_ALIGN               (-13)
#define LPP_ERR_NULL_PTR            (-14)

#define LPP_FLAGS_IGNORE_CFGEXPR_MSK    (1<<0)
#define LPP_FLAGS_IGNORE_CFGEXPR_OFF    (0<<0)
#define LPP_FLAGS_IGNORE_CFGEXPR_ON     (1<<0)

#define LPP_FLAGS_SAT_TYPE_MSK          (3<<1)
#define LPP_FLAGS_SAT_TYPE_ANY          (0<<1)  /* Constant indicating we're searching for any type of satellite */
#define LPP_FLAGS_SAT_TYPE_RX           (1<<1)  /* Constant indicating we're searching for a receiving satellite */
#define LPP_FLAGS_SAT_TYPE_TX           (2<<1)  /* Constant indicating we're searching for a transmitting satellite */

#define LPP_DEFAULT_MIN_ELEVATION       50
#define LPP_DEFAULT_LOOKAHEAD           (60 * 60 * 24 * 7)  /* One week */

typedef enum {
    lpp_lrfhss_coding_rate_5_6 = 0,
    lpp_lrfhss_coding_rate_2_3 = 1,
    lpp_lrfhss_coding_rate_1_2 = 2,
    lpp_lrfhss_coding_rate_1_3 = 3
} lpp_lrfhss_coding_rate;

typedef enum {
    lpp_lrfhss_grid_25_khz = 0,
    lpp_lrfhss_grid_3_9_khz = 1
} lpp_lrfhss_grid;

typedef enum {
	lpp_lrfhss_bandwidth_39_06_khz = 0,
	lpp_lrfhss_bandwidth_89_84_khz = 1,
	lpp_lrfhss_bandwidth_136_7_khz = 2,
	lpp_lrfhss_bandwidth_187_5_khz = 3,
	lpp_lrfhss_bandwidth_335_9_khz = 4,
	lpp_lrfhss_bandwidth_386_7_khz = 5,
	lpp_lrfhss_bandwidth_722_6_khz = 6,
	lpp_lrfhss_bandwidth_773_4_khz = 7,
	lpp_lrfhss_bandwidth_1523_4_khz = 8,
	lpp_lrfhss_bandwidth_1574_2_khz = 9
} lpp_lrfhss_bandwidth;

typedef struct {
    lpp_lrfhss_coding_rate coding_rate;
    lpp_lrfhss_bandwidth bandwidth;
    lpp_lrfhss_grid grid;
    uint8_t hopping_on;
    uint8_t nr_sync;
    uint32_t frequency;
    int8_t power;
    uint16_t time_spread;
} lpp_lrfhss_tx_config;

typedef enum {
	lpp_lora_spreading_factor_5 = 5,
	lpp_lora_spreading_factor_6 = 6,
	lpp_lora_spreading_factor_7 = 7,
	lpp_lora_spreading_factor_8 = 8,
	lpp_lora_spreading_factor_9 = 9,
	lpp_lora_spreading_factor_10 = 10,
	lpp_lora_spreading_factor_11 = 11,
	lpp_lora_spreading_factor_12 = 12
} lpp_lora_spreading_factor;

typedef enum {
	lpp_lora_bandwidth_7_81_khz = 0,
	lpp_lora_bandwidth_10_41_khz = 1,
	lpp_lora_bandwidth_15_62_khz = 2,
	lpp_lora_bandwidth_20_83_khz = 3,
	lpp_lora_bandwidth_31_25_khz = 4,
	lpp_lora_bandwidth_41_66_khz = 5,
	lpp_lora_bandwidth_62_50_khz = 6,
	lpp_lora_bandwidth_125_khz = 7,
	lpp_lora_bandwidth_250_khz = 8,
	lpp_lora_bandwidth_500_khz = 9
} lpp_lora_bandwidth;

typedef struct {
    lpp_lora_spreading_factor spreading_factor;
    uint8_t invert_iq;
    uint8_t ldro;
    uint32_t frequency;
    lpp_lora_bandwidth bandwidth;
    uint16_t sync_word;
    uint32_t preamble_length;
} lpp_lora_rx_config;

typedef enum {
    lpp_contact_type_rx = 0,    /* A contact with a receiving satellite (so the sensor can transmit) */
    lpp_contact_type_tx = 1     /* A contact with a transmitting satellite (so the sensor can receive) */
} lpp_contact_type;

typedef struct {
    uint8_t sat_id;             /* Numeric ID of the satellite */
    uint32_t tca;               /* Time of closest approach, in seconds since the Unix epoch */
    uint32_t contact_start;     /* Time at which the elevation rises above the minimum elevation, in seconds
                                   since the Unix epoch */
    uint32_t contact_end;       /* Time at which the elevation drops below the minimum elevation in seconds
                                   since the Unix epoch */
    double pca_lon, pca_lat;    /* Longitude and latitude of the position of closest approach, in degrees */
    double max_elevation;       /* The maximum elevation reached during the contact, in degrees */

    lpp_contact_type contact_type;  /* The type of contact (with a receiving satellite or with a transmitting satellite) */
    union {
        lpp_lrfhss_tx_config tx;    /* The transmit configuration; filled in when the contact is of type lpp_contact_type_rx */
        lpp_lora_rx_config rx;      /* The receive configuration; filled in when the contact is of type lpp_contact_type_tx */
    } config;                   
} lpp_contact;

/*
 * Setup function initializing the LPP datastore. This function has to be called once before starting to use the LPP library.
 * By calling this function, the application gives the LPP library access to a region of memory that the LPP library can
 * use for its own storage. On all subsequent calls to lpp functions, this same data has to be passed in. The location of the
 * data may change between calls, as long as alignment restrictions are observed. It is also permitted for an application
 * to write out the data to external storage and load it in before a next call to an lpp function.
 * @param data pointer to the beginning of memory to be used by lpp. Must be aligned to 4 bytes.
 * @param size size of the area of memory. Recommended value is about 4KB.
 * @return LPP_ERR_OK if successful, an error < 0 on error.
 */
int32_t lpp_setup(uint8_t *data, size_t size);

/*
 * Add any message received from a transmitting satellite (or read from file) to the LPP data-store.
 * @param data memory reserved for lpp, initialized earlier by a call to `lpp_setup` and aligned
 *        to 4 bytes.
 * @param records data as received from a transmitting satellite (or read from file). The data is expected
 *        to start with value `0xe4`, indicating that it contains lpp records.
 * @param record_size number of bytes in `data`
 * @return LPP_ERR_OK if successful, an error < 0 on error.
 */
int32_t lpp_add_records(uint8_t *data, uint8_t *records, size_t records_size);


/*
 * Calculates the next time a satellite is visible and available from the caller's location, and returns 
 * the configuration needed to send to/receive from it.
 * @param data memory reserved for lpp, initialized earlier by a call to `lpp_setup` and aligned
 *        to 4 bytes.
 * @param search_start time (in seconds after Unix epoch) after which to find a contact.
 * @param lookahead duration (in seconds) after search_start within which the contact is to be found.
 * @param lon caller's longitude in degrees
 * @param lat caller's latitude in degrees
 * @param min_elevation minimum required elevation in degrees. Use LPP_MIN_DEFAULT_ELEVATION unless you have 
 *        special requirements.
 * @param contact_data pointer to an lpp_contact struct that will receive the result in case a contact is found.
 * @param flags combination of flags modifying the searchbehaviour. The only usable flags are:
 *        - LPP_FLAGS_SAT_TYPE_RX to only find receiving satellites.
 *        - LPP_FLAGS_SAT_TYPE_TX to only find transmitting satellites.
 *        - LPP_FLAGS_SAT_TYPE_ANY to find bith receiving and transmitting satellites (this is the default)
 * @return LPP_ERR_OK if a contact was found. In this case the contact details are in the struct pointed at by contact_data.
 *         LPP_ERR_NOT_FOUND if no contact could be found within the given lookahead seconds.
 *         Any other error < 0 if an error occurred.
 */
int32_t lpp_next_contact(uint8_t *data, uint32_t search_start, uint32_t lookahead, double lon, double lat, double min_elevation, lpp_contact *contact_data, uint32_t flags);

/*
 * Get the age of the oldest and the newest record in the lpp store.
 * @param data memory reserved for lpp, initialized earlier by a call to `lpp_setup` and aligned
 *        to 4 bytes.
 * @param oldest_record pointer to uint32_t that will receive the start-time of the oldest record
 *        in the store
 * @param newest_record pointer to uint32_t that will receive the start-time of the newest record
 *        in the store
 * @return LPP_ERR_OK if a result was found. In this case the result is returned in oldest_record
 *                    and newest_record.
 *         LPP_ERR_NOT_FOUND if the store does not contain any records at all
 *         Any other error < 0 if an error ocurred.
 */
int32_t lpp_get_records_age(uint8_t *data, uint32_t *oldest_record, uint32_t *newest_record);

/*
 * Translate an error-code as returned by an lpp_ function to a human-readable message.
 * @param error the error-code to be translated
 * @return a 0-terminated string containing a human-readable translation. The message is
 *         located in statically allocated memory and must not be modified nor deallocated.
 */
char *lpp_error_to_string(int32_t error);


#ifdef __cplusplus
}
#endif

#endif
