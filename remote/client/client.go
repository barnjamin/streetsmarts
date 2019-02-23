package main

import (
	"fmt"
	"log"
	"net/http"
	"os"
	"time"
)

type DataType string

var (
	DEPTH    DataType = "depth"
	COLOR    DataType = "color"
	FRAGMENT DataType = "fragment"
	GPS      DataType = "gps"
	IMU      DataType = "imu"
)

var (
	host = "127.0.0.1"
	port = "8080"

	remote = fmt.Sprintf("http://%s:%s", host, port)

	client = &http.Client{}
)

func main() {

	// Start a session on the server
	session, err := StartSession()
	if err != nil {
		log.Fatalf("Failed to start session: %+v", err)
	}

	log.Printf("Got a session id of: %s", session)

	// Create local directory for rgbd images/fragments/imu/gps
	// Watch for new files and upload them to server
	// When the program exits(?) stop session
	// Start polling for finished posegraph
	// On finished posegraph, integrate final scene
	// Send final scene to server
	// exit
}

func StartSession() (string, error) {
	req, err := http.NewRequest("POST", remote+"/start", nil)
	if err != nil {
		log.Printf("Failed to Create request for remote server: %+v", err)
		return "", err
	}
	req.Header.Add("id", "abc123") //TODO

	resp, err := client.Do(req)
	if err != nil {
		log.Printf("Failed to create session on server: %+v", err)
		return "", err
	}

	//Get the session id we're going to use in later requests
	return resp.Header.Get("session-id"), nil
}

func PrepareDirs(session_id string) error {
	session := fmt.Sprintf("%s-%d", id, time.Now().Unix())

	gps_session_dir := get_session_dir(session, GPS)
	if err := os.MkdirAll(gps_session_dir, 0777); err != nil {
		return session, err
	}

	imu_session_dir := get_session_dir(session, IMU)
	if err := os.MkdirAll(imu_session_dir, 0777); err != nil {
		return session, err
	}

	fragment_session_dir := get_session_dir(session, FRAGMENT)
	if err := os.MkdirAll(fragment_session_dir, 0777); err != nil {
		return session, err
	}

	fragment_session_dir := get_session_dir(session, COLOR)
	if err := os.MkdirAll(fragment_session_dir, 0777); err != nil {
		return session, err
	}

	fragment_session_dir := get_session_dir(session, DEPTH)
	if err := os.MkdirAll(fragment_session_dir, 0777); err != nil {
		return session, err
	}

	return session, nil
}

func get_session_dir(session_id string, datatype DataType) string {
	return fmt.Sprintf("/home/ben/local-sessions/%s/%s", session_id, datatype)
}
