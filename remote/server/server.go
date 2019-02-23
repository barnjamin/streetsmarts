package main

import (
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"os/exec"
	"time"
)

type DataType string

var (
	GPS      DataType = "gps"
	IMU      DataType = "imu"
	FRAGMENT DataType = "fragment"
	FINAL    DataType = "final"
)

func main() {
	http.HandleFunc("/start", Start)
	http.HandleFunc("/stop", Stop)

	http.HandleFunc("/fragment", UploadFragment)
	http.HandleFunc("/imu", UploadIMU)
	http.HandleFunc("/gps", UploadGPS)

	http.HandleFunc("/check", CheckFinished)

	http.HandleFunc("/final", UploadFinal)

	log.Fatal(http.ListenAndServe(":8080", nil))
}

// Start starts a session for keeping local files together
func Start(w http.ResponseWriter, r *http.Request) {
	// Get ID of client, hardcoded on client to associate it with a customer
	id := r.Header.Get("id")

	// Start a session by creating directories and generating
	// a session id for later use by client
	session, err := StartSession(id)
	if err != nil {
		log.Printf("Failed to create session")
		w.WriteHeader(500)
		return
	}

	// Send back session id
	w.Header().Add("session-id", session)
	w.WriteHeader(http.StatusOK)
}

func Stop(w http.ResponseWriter, r *http.Request) {
	//Stop session
	// Write some sentinal to show that we've finished
	// Kick off processor?
}

func UploadFragment(w http.ResponseWriter, r *http.Request) {
	session := r.Header.Get("session-id")
	fragment := r.Header.Get("fragment")

	path := fmt.Sprintf("%s/%s.ply.drc", get_session_dir(session, FRAGMENT), fragment)
	f, err := os.Create(path)
	if err != nil {
		log.Printf("Failed to create file")
		w.WriteHeader(500)
		return
	}

	defer f.Close()
	defer r.Body.Close()

	// write body to directory of session given in headers
	if _, err = io.Copy(f, r.Body); err != nil {
		log.Printf("Failed to write file")
		w.WriteHeader(500)
		return
	}

	go func() {
		// decompress with draco via commandline
		cmd := exec.Command("/home/ben/draco/build/draco_decode", "-i", path, "-o", path[:3])
		out, err := cmd.CombinedOutput()
		if err != nil {
			log.Printf("Failed to decompress: %s", path)
			return
		}

		log.Printf("Output from draco: %s", out)

		os.Remove(path)
	}()

	w.WriteHeader(http.StatusOK)
	return
}

func UploadIMU(w http.ResponseWriter, r *http.Request) {
	// write body to directory of session given in headers with .imu extension
	// return success
}

func UploadGPS(w http.ResponseWriter, r *http.Request) {
	// write body to directory of session given in headers with .gps extension
	// return success
}

func CheckFinished(w http.ResponseWriter, r *http.Request) {
	// Client is asking for final posegraph or status on finshed state
}

func UploadFinal(w http.ResponseWriter, r *http.Request) {
	// Client sending final mesh file
}

func StartSession(id string) (string, error) {
	session := fmt.Sprintf("%s-%d", id, time.Now().Unix())

	gps_session_dir := get_session_dir(session, GPS)
	if err := os.Mkdir(gps_session_dir, 0666); err != nil {
		return session, err
	}

	imu_session_dir := get_session_dir(session, IMU)
	if err := os.Mkdir(imu_session_dir, 0666); err != nil {
		return session, err
	}

	fragment_session_dir := get_session_dir(session, FRAGMENT)
	if err := os.Mkdir(fragment_session_dir, 0666); err != nil {
		return session, err
	}

	final_session_dir := get_session_dir(session, FINAL)
	if err := os.Mkdir(final_session_dir, 0666); err != nil {
		return session, err
	}

	return session, nil
}

func get_session_dir(session_id string, data_type DataType) string {
	return fmt.Sprintf("/home/ben/sessions/%s/%s", session_id, data_type)
}
