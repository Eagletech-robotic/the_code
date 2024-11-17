// ----------------------
// INSTALLATION
// ----------------------
//
// sudo apt-get install libcjson-dev
// ./MAKE
// ./build/http_server
//
// ----------------------
// POINTS D'ENTRÉE (ENDPOINTS):
// ----------------------
//
// Init/reset (À IMPLÉMENTER)
//   Requête :
//     curl -X POST -d
//     '{"person":{"name":"Alice","age":30},"location":{"city":"Paris","country":"France"}}'
//     http://localhost:8080/init
//   Réponse :
//     {"message":"Bienvenue Alice de Paris, France!"}
//
// Step
//   Requête :
//     curl -X POST -d '{
//         "is_jack_gone": 1,
//         "tof": 1234.5,
//         "gyro": [0.1, 0.2, 0.3],
//         "accelero": [0.01, 0.02, 0.03],
//         "compass": [10.1, 10.2, 10.3],
//         "last_wifi_data": [1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
//         "encoder1": 150,
//         "encoder2": 200
//     }' http://localhost:8080/step
//   Réponse :
//     {
//         "vitesse1_ratio": 0.85,
//         "vitesse2_ratio": 0.90,
//         "servo_pelle_ratio": 1.0
//     }
//
//  Pour le moment (sep 2024), seuls ces paramètres sont lus en entrée :
//    curl -X POST -d '{"encoder1": 110, "encoder2": 90}' http://localhost:8080/step

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cjson/cJSON.h"
#include "iot01A/top_driver.h"

#define PORT 8080

// Structures pour /init
typedef struct {
    char name[50];
    int age;
    char city[50];
    char country[50];
} query_init_t;
typedef struct {
    char welcome_message[1000];
} response_init_t;

// Fonction myinit()
void myinit(query_init_t *query_init, response_init_t *response_init) {
    sprintf(response_init->welcome_message, "Bienvenue %s de %s, %s!", query_init->name,
            query_init->city, query_init->country);
}

// Fonction mystep()
void mystep(input_t *input, output_t *output) {
    // Traitement des données d'entrée pour produire la sortie
    // Exemple de logique (à adapter selon vos besoins) :

    // Calculer ratio2 en fonction de tof (Time of Flight)
    /* output->ratio2 = input->tof / 1000.0f; // Exemple : normaliser la
     * distance
     */

    /* // Calculer ratio15 en fonction des données gyroscopiques */
    /* output->ratio15 = (input->gyro[0] + input->gyro[1] + input->gyro[2])
     * / 3.0f; */

    /* // Déterminer servo_pelle_ratio en fonction de is_jack_gone */
    /* output->servo_pelle_ratio = input->is_jack_gone ? 1.0f : 0.0f; */
    config_t config;
    config.time_step_ms = 5;
    top_step(&config, input, output);
}

void output_to_json(output_t *output, cJSON *json) {
    if (output == NULL || json == NULL) {
        return;
    }

    cJSON_AddNumberToObject(json, "vitesse1_ratio", output->vitesse1_ratio);

    cJSON_AddNumberToObject(json, "vitesse1_ratio", output->vitesse2_ratio);

    cJSON_AddNumberToObject(json, "servo_pelle_ratio", output->servo_pelle_ratio);
}

void decode_step_input_int(cJSON *json, const char *key, int *target) {
    cJSON *item = cJSON_GetObjectItem(json, key);
    if (item && cJSON_IsNumber(item)) {
        *target = item->valueint;
    }
    *target = 0;
}

void decode_step_input_float(cJSON *json, const char *key, float *target) {
    cJSON *item = cJSON_GetObjectItem(json, key);
    if (item && cJSON_IsNumber(item)) {
        *target = (float)item->valuedouble;
    }
    *target = 0.0;
}

void decode_step_input_float_array(cJSON *json, const char *key, float array[], int size) {
    cJSON *item = cJSON_GetObjectItem(json, key);
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < size; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                array[i] = (float)item->valuedouble;
            } else {
                array[i] = 0.0;
            }
        }
    }
}

void decode_step_input_int_array(cJSON *json, const char *key, int array[], int size) {
    cJSON *item = cJSON_GetObjectItem(json, key);
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < size; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                array[i] = item->valueint;
            } else {
                array[i] = 0;
            }
        }
    }
}

void decode_step_input(cJSON *json, input_t *input) {
    cJSON *item = NULL;

    decode_step_input_int(json, "is_jack_gone", &input->is_jack_gone);
    decode_step_input_float(json, "tof", &input->tof);
    decode_step_input_float_array(json, "gyro", input->gyro, 3);
    decode_step_input_float_array(json, "accelero", input->accelero, 3);
    decode_step_input_float_array(json, "compass", input->compass, 3);
    decode_step_input_int_array(json, "last_wifi_data", input->last_wifi_data, 10);
    decode_step_input_int(json, "encoder1", &input->encoder1);
    decode_step_input_int(json, "encoder2", &input->encoder2);
}

void handle_options(int client_socket) {
    // Send a response with allowed methods
    char *response =
        "HTTP/1.1 204 No Content\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Content-Length: 0\r\n"
        "\r\n";
    write(client_socket, response, strlen(response));
    close(client_socket);
}

void handle_init(int client_socket, const char *json_data) {
    // Parse the JSON data
    cJSON *json = cJSON_Parse(json_data);
    if (!json) {
        close(client_socket);
        return;
    }

    // Extraire les données dans la structure query_init_t
    query_init_t query_init;
    memset(&query_init, 0, sizeof(query_init_t));

    cJSON *json_person = cJSON_GetObjectItem(json, "person");
    cJSON *json_location = cJSON_GetObjectItem(json, "location");

    if (json_person) {
        cJSON *name = cJSON_GetObjectItem(json_person, "name");
        cJSON *age = cJSON_GetObjectItem(json_person, "age");
        if (name && age) {
            strcpy(query_init.name, name->valuestring);
            query_init.age = age->valueint;
        }
    }

    if (json_location) {
        cJSON *city = cJSON_GetObjectItem(json_location, "city");
        cJSON *country = cJSON_GetObjectItem(json_location, "country");
        if (city && country) {
            strcpy(query_init.city, city->valuestring);
            strcpy(query_init.country, country->valuestring);
        }
    }

    // Appel de la fonction myinit()
    response_init_t response_init;
    memset(&response_init, 0, sizeof(response_init_t));
    myinit(&query_init, &response_init);

    // Créer le JSON de réponse
    cJSON *response_json = cJSON_CreateObject();
    cJSON_AddStringToObject(response_json, "message", response_init.welcome_message);

    char *response_data = cJSON_Print(response_json);

    // Envoyer la réponse HTTP
    char response[2048];
    sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %lu\r\n"
            "\r\n"
            "%s",
            strlen(response_data), response_data);

    write(client_socket, response, strlen(response));

    // Nettoyage
    cJSON_Delete(json);
    cJSON_Delete(response_json);
    free(response_data);
    close(client_socket);
}

void handle_step(int client_socket, const char *json_data) {
    // Parse the JSON data from the request
    cJSON *json_request = cJSON_Parse(json_data);
    if (!json_request) {
        // Envoyer une réponse d'erreur
        const char *error_response = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
        write(client_socket, error_response, strlen(error_response));
        close(client_socket);
        return;
    }

    // Convertir le JSON en structure input_t
    input_t input_data;
    memset(&input_data, 0, sizeof(input_t));
    decode_step_input(json_request, &input_data);

    // Appeler mystep() pour traiter les données et obtenir output_t
    output_t output_data;
    memset(&output_data, 0, sizeof(output_t));
    mystep(&input_data, &output_data);

    // Convertir la structure output_t en JSON pour la réponse
    cJSON *json_response = cJSON_CreateObject();
    output_to_json(&output_data, json_response);

    char *response_data = cJSON_Print(json_response);

    // Envoyer la réponse HTTP
    char response[4096];
    sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %lu\r\n"
            "\r\n"
            "%s",
            strlen(response_data), response_data);

    write(client_socket, response, strlen(response));

    // Nettoyage
    cJSON_Delete(json_request);
    cJSON_Delete(json_response);
    free(response_data);
    close(client_socket);
}
void handle_client(int client_socket) {
    char buffer[2048] = {0};
    read(client_socket, buffer, 2048);

    // Parse the HTTP request
    char method[10];
    char url[100];
    sscanf(buffer, "%s %s", method, url);

    // Simple HTTP request parsing (assuming POST request with JSON body)
    char *json_start = strstr(buffer, "\r\n\r\n");
    if (!json_start) {
        close(client_socket);
        return;
    }
    json_start += 4;  // Move past the "\r\n\r\n"

    if (strcmp(method, "OPTIONS") == 0) {
        handle_options(client_socket);
    } else if (strcmp(url, "/init") == 0 && strcmp(method, "POST") == 0) {
        handle_init(client_socket, json_start);
    } else if (strcmp(url, "/step") == 0 && strcmp(method, "POST") == 0) {
        handle_step(client_socket, json_start);
    } else {
        // Send 404 Not Found
        char *response =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Length: 0\r\n"
            "\r\n";
        write(client_socket, response, strlen(response));
        close(client_socket);
    }
}

int main() {
    int server_fd, client_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Créer le socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Échec de la création du socket");
        exit(EXIT_FAILURE);
    }

    // Attacher le socket au port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("Erreur setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Lier le socket
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("Échec du bind");
        exit(EXIT_FAILURE);
    }

    // Écouter les connexions entrantes
    if (listen(server_fd, 3) < 0) {
        perror("Échec de l'écoute");
        exit(EXIT_FAILURE);
    }

    printf("Serveur en écoute sur le port %d\n", PORT);

    while (1) {
        // Accepter une nouvelle connexion
        if ((client_socket =
                 accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
            perror("Échec de l'acceptation");
            exit(EXIT_FAILURE);
        }

        // Gérer le client
        handle_client(client_socket);
    }

    return 0;
}
