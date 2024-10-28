// sudo apt-get install libcjson-dev
// gcc -o serveur serveur.c -lcjson
// ./server
//curl -X POST -H "Content-Type: application/json" -d '{"person":{"name":"Alice","age":30},"location":{"city":"Paris","country":"France"}}' http://localhost:8080/init
// curl -X POST -H "Content-Type: application/json" -d '{"step":1,"action":"start"}' http://localhost:8080/step

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include "cjson/cJSON.h"
#include "output.h"
#include "input.h"

#define PORT 8080

// Structures pour /init
typedef struct {
    char name[50];
    int age;
    char city[50];
    char country[50];
} query_init_t;

typedef struct {
    char welcome_message[100];
} response_init_t;

// Structures pour /step
typedef struct {
    int step;
    char action[50];
} query_step_t;

typedef struct {
    int next_step;
    char status[100];
} response_step_t;

// Fonction myinit()
void myinit(query_init_t *query_init, response_init_t *response_init) {
    sprintf(response_init->welcome_message, "Bienvenue %s de %s, %s!", query_init->name, query_init->city, query_init->country);
}

// Fonction mystep()
void mystep(input_t *input, output_t *output) {
    // Traitement des données d'entrée pour produire la sortie
    // Exemple de logique (à adapter selon vos besoins) :

    // Calculer ratio2 en fonction de tof (Time of Flight)
    output->ratio2 = input->tof / 1000.0f; // Exemple : normaliser la distance

    // Calculer ratio15 en fonction des données gyroscopiques
    output->ratio15 = (input->gyro[0] + input->gyro[1] + input->gyro[2]) / 3.0f;

    // Déterminer servo_pelle_ratio en fonction de is_jack_gone
    output->servo_pelle_ratio = input->is_jack_gone ? 1.0f : 0.0f;
}

void output_to_json(output_t *output, cJSON *json) {
    if (output == NULL || json == NULL) {
        return;
    }

    // Ajouter "ratio2"
    cJSON_AddNumberToObject(json, "ratio2", output->ratio2);

    // Ajouter "ratio15"
    cJSON_AddNumberToObject(json, "ratio15", output->ratio15);

    // Ajouter "servo_pelle_ratio"
    cJSON_AddNumberToObject(json, "servo_pelle_ratio", output->servo_pelle_ratio);
}


void json_to_input(cJSON *json, input_t *input) {
    cJSON *item = NULL;

    // Récupérer "is_jack_gone"
    item = cJSON_GetObjectItem(json, "is_jack_gone");
    if (item && cJSON_IsNumber(item)) {
        input->is_jack_gone = item->valueint;
    } else {
        input->is_jack_gone = 0; // Valeur par défaut ou gérer l'erreur
    }

    // Récupérer "tof"
    item = cJSON_GetObjectItem(json, "tof");
    if (item && cJSON_IsNumber(item)) {
        input->tof = (float)item->valuedouble;
    } else {
        input->tof = 0.0f;
    }

    // Récupérer "gyro"
    item = cJSON_GetObjectItem(json, "gyro");
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < 3; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                input->gyro[i] = (float)subitem->valuedouble;
            } else {
                input->gyro[i] = 0.0f;
            }
        }
    }

    // Récupérer "accelero"
    item = cJSON_GetObjectItem(json, "accelero");
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < 3; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                input->accelero[i] = (float)subitem->valuedouble;
            } else {
                input->accelero[i] = 0.0f;
            }
        }
    }

    // Récupérer "compass"
    item = cJSON_GetObjectItem(json, "compass");
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < 3; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                input->compass[i] = (float)subitem->valuedouble;
            } else {
                input->compass[i] = 0.0f;
            }
        }
    }

    // Récupérer "last_wifi_data"
    item = cJSON_GetObjectItem(json, "last_wifi_data");
    if (item && cJSON_IsArray(item)) {
        for (int i = 0; i < 10; i++) {
            cJSON *subitem = cJSON_GetArrayItem(item, i);
            if (subitem && cJSON_IsNumber(subitem)) {
                input->last_wifi_data[i] = subitem->valueint;
            } else {
                input->last_wifi_data[i] = 0;
            }
        }
    }
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
    json_to_input(json_request, &input_data);

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
    json_start += 4; // Move past the "\r\n\r\n"

    if (strcmp(url, "/init") == 0) {
        handle_init(client_socket, json_start);
    } else if (strcmp(url, "/step") == 0) {
        handle_step(client_socket, json_start);
    } else {
        // Send 404 Not Found
        char *response = "HTTP/1.1 404 Not Found\r\n"
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
        if ((client_socket = accept(server_fd, (struct sockaddr *)&address,
                                    (socklen_t *)&addrlen)) < 0) {
            perror("Échec de l'acceptation");
            exit(EXIT_FAILURE);
        }

        // Gérer le client
        handle_client(client_socket);
    }

    return 0;
}
