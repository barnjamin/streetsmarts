package main

import (
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"os/exec"
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
	//log.Printf("Starting a session")
	//session, err := StartSession()
	//if err != nil {
	//	log.Fatalf("Failed to start session: %+v", err)
	//}

	//log.Printf("Got a session id of: %s", session)
	//if err := PrepareDirs(session); err != nil {
	//	log.Fatalf("Failed to prepare directory: %+v", err)
	//}

	//log.Printf("Prepared Directories for writing")

	// TODO: Kick off program to capture rgbd images and generate fragments

	session := "abc123-1550930895"
	// Watch for new files and upload them to server
	WatchDir(session, FRAGMENT)
	//WatchDir(session, IMU)
	//WatchDir(session, GPS)

	// When the program exits(?) stop session

	// Start polling for finished posegraph

	// On finished posegraph, integrate final scene

	// Send final scene to server
}

func WatchDir(session string, datatype DataType) {
	//https://github.com/fsnotify/fsnotify/blob/master/example_test.go
	//watch_dir := get_session_dir(session, datatype)

	compress := false

	watch_dir := "/home/ben/streetsmarts/dumps/latest/fragments_cuda"
	files, err := ioutil.ReadDir(watch_dir)
	if err != nil {
		log.Fatalf("Failed to watch directory")
	}

	for _, file := range files {
		fname := file.Name()

		//Skip non ply files
		if fname[len(fname)-3:] != "ply" {
			continue
		}

		if compress {
			// Compress to fs
			cmd := exec.Command("/home/ben/draco/build/draco_encode", "-i", watch_dir+"/"+fname)
			out, err := cmd.CombinedOutput()
			if err != nil {
				log.Printf("Failed to encode: %s", fname)
				log.Printf("Output from draco: %s", out)
				continue
			}
			fname += ".drc"
		}

		f, err := os.Open(watch_dir + "/" + fname)
		if err != nil {
			log.Printf("Failed to open compressed file: %+v", err)
		}

		req, err := http.NewRequest("POST", remote+"/fragment", f)
		if err != nil {
			log.Printf("Failed to create new request: %+v", err)
			continue
		}

		req.Header.Add("session-id", session)
		req.Header.Add("fragment", fname)

		// Write Compressed to server
		_, err = client.Do(req)
		if err != nil {
			log.Printf("Failed to upload fragment: %+v", err)
			continue
		}

		// TODO: Delete both
	}
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

func PrepareDirs(session string) error {
	// Create local directory for rgbd images/fragments/imu/gps
	gps_session_dir := get_session_dir(session, GPS)
	if err := os.MkdirAll(gps_session_dir, 0777); err != nil {
		return err
	}

	imu_session_dir := get_session_dir(session, IMU)
	if err := os.MkdirAll(imu_session_dir, 0777); err != nil {
		return err
	}

	fragment_session_dir := get_session_dir(session, FRAGMENT)
	if err := os.MkdirAll(fragment_session_dir, 0777); err != nil {
		return err
	}

	color_session_dir := get_session_dir(session, COLOR)
	if err := os.MkdirAll(color_session_dir, 0777); err != nil {
		return err
	}

	depth_session_dir := get_session_dir(session, DEPTH)
	if err := os.MkdirAll(depth_session_dir, 0777); err != nil {
		return err
	}

	return nil
}

func get_session_dir(session_id string, datatype DataType) string {
	return fmt.Sprintf("/home/ben/local-sessions/%s/%s", session_id, datatype)
}
