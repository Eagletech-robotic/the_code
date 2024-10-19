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

#define PORT 8080

typedef struct {
    char name[50];
    int age;
} Person;

typedef struct {
    char city[50];
    char country[50];
} Location;

void handle_init(int client_socket, const char *json_data) {
    // Parse the JSON data
    cJSON *json = cJSON_Parse(json_data);
    if (!json) {
        close(client_socket);
        return;
    }

    // Extract data into structures
    Person person;
    Location location;

    cJSON *json_person = cJSON_GetObjectItem(json, "person");
    cJSON *json_location = cJSON_GetObjectItem(json, "location");

    if (json_person) {
        cJSON *name = cJSON_GetObjectItem(json_person, "name");
        cJSON *age = cJSON_GetObjectItem(json_person, "age");
        if (name && age) {
            strcpy(person.name, name->valuestring);
            person.age = age->valueint;
        }
    }

    if (json_location) {
        cJSON *city = cJSON_GetObjectItem(json_location, "city");
        cJSON *country = cJSON_GetObjectItem(json_location, "country");
        if (city && country) {
            strcpy(location.city, city->valuestring);
            strcpy(location.country, country->valuestring);
        }
    }

    // Create response JSON
    cJSON *response_json = cJSON_CreateObject();
    cJSON_AddStringToObject(response_json, "message", "Init data received successfully");
    cJSON_AddItemToObject(response_json, "person", cJSON_Duplicate(json_person, 1));
    cJSON_AddItemToObject(response_json, "location", cJSON_Duplicate(json_location, 1));

    char *response_data = cJSON_Print(response_json);

    // Send HTTP response
    char response[2048];
    sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %lu\r\n"
            "\r\n"
            "%s",
            strlen(response_data), response_data);

    write(client_socket, response, strlen(response));

    // Clean up
    cJSON_Delete(json);
    cJSON_Delete(response_json);
    free(response_data);
    close(client_socket);
}

void handle_step(int client_socket, const char *json_data) {
    // Parse the JSON data
    cJSON *json = cJSON_Parse(json_data);
    if (!json) {
        close(client_socket);
        return;
    }

    // Extract data
    int step = 0;
    char action[50] = {0};

    cJSON *json_step = cJSON_GetObjectItem(json, "step");
    cJSON *json_action = cJSON_GetObjectItem(json, "action");

    if (json_step && json_action) {
        step = json_step->valueint;
        strcpy(action, json_action->valuestring);
    }

    // Create response JSON
    cJSON *response_json = cJSON_CreateObject();
    cJSON_AddStringToObject(response_json, "message", "Step data received successfully");
    cJSON_AddNumberToObject(response_json, "step", step);
    cJSON_AddStringToObject(response_json, "action", action);

    char *response_data = cJSON_Print(response_json);

    // Send HTTP response
    char response[2048];
    sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %lu\r\n"
            "\r\n"
            "%s",
            strlen(response_data), response_data);

    write(client_socket, response, strlen(response));

    // Clean up
    cJSON_Delete(json);
    cJSON_Delete(response_json);
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
