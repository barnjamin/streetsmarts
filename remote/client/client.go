package main

import (
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"os/exec"
	"sync"
	"time"
)

type DataType string

var (
	DEPTH    DataType = "depth"
	COLOR    DataType = "color"
	FRAGMENT DataType = "fragment"
	GPS      DataType = "gps"
	IMU      DataType = "imu"
	POSE     DataType = "pose"
)

var (
	host = "127.0.0.1"
	port = "8080"

	remote = fmt.Sprintf("http://%s:%s", host, port)

	client = &http.Client{}

	compress = flag.Bool("compress", false, "Whether or not to apply draco compression")

	finished = false
)

func main() {
	flag.Parse()

	// Start a session on the server
	log.Printf("Starting a session")
	session, err := StartSession()
	if err != nil {
		log.Fatalf("Failed to start session: %+v", err)
	}

	log.Printf("Got a session id of: %s", session)
	if err := PrepareDirs(session); err != nil {
		log.Fatalf("Failed to prepare directory: %+v", err)
	}

	log.Printf("Prepared Directories for writing")

	var wg sync.WaitGroup

	wg.Add(1)
	go func() {
		defer wg.Done()

		log.Printf("Kicking off data capture")

		time.Sleep(3 * time.Second)

		//cmd := exec.Command("/home/ben/streetsmarts/build/bin/capture", session)
		//out, err := cmd.CombinedOutput()
		//if err != nil {
		//  log.Printf("Output: %s", out)
		//	log.Fatalf("Failed to run command")
		//}

		//log.Printf("Output: %s", out)
	}()

	// Watch for new files and upload them to server
	log.Printf("Watching directories")
	WatchDir(session, FRAGMENT, file_wg)
	WatchDir(session, POSE, file_wg)
	//WatchDir(session, IMU, file_wg)
	//WatchDir(session, GPS, file_wg)

	log.Printf("Waiting for capture program to terminate")
	wg.Wait()

	log.Printf("Capture program finished, waiting for last files to be uploaded")
	CleanUp()

	log.Printf("Stopping session")
	if err := StopSession(session); err != nil {
		log.Fatalf("Failed to stop session %q: %+v", session, err)
	}

	log.Printf("Waiting for finished posegraph")
	// Start polling for finished posegraph
	if err := CheckFinished(session); err != nil {
		log.Fatalf("Couldnt check if session %s is finished: %s", session, err)
	}

	log.Printf("Integrating scene")

	// On finished posegraph, integrate final scene
	// Send final scene to server
}

func CleanUp() {
	finished = true
}

func WatchDir(session string, datatype DataType) {
	//https://github.com/fsnotify/fsnotify/blob/master/example_test.go
	//watch_dir := get_session_dir(session, datatype)
	watch_dir := "/home/ben/streetsmarts/dumps/latest/fragments_cuda"
	go func() {
		for {
			//Run it one more time
			last_run := false
			if finished {
				last_run = true
			}

			switch datatype {
			case FRAGMENT:
				watchPLY(session, watch_dir)
			case POSE:
				watchPose(session, watch_dir)
			}

			if last_run {
				return
			}
		}
	}()

}

func watchPose(session, watch_dir string) {
	files, err := ioutil.ReadDir(watch_dir)
	if err != nil {
		log.Fatalf("Failed to watch directory")
	}

	for _, file := range files {
		fname := file.Name()

		//Skip non ply files
		if fname[len(fname)-4:] != "json" {
			continue
		}

		f, err := os.Open(watch_dir + "/" + fname)
		if err != nil {
			log.Printf("Failed to open compressed file: %+v", err)
		}

		req, err := http.NewRequest("POST", remote+"/pose", f)
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

		//TODO: delete
	}
}

func watchPLY(session, watch_dir string) {

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

		if *compress {
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

func StopSession(session string) error {

	req, err := http.NewRequest("POST", remote+"/stop", nil)
	if err != nil {
		log.Printf("Failed to create request for remote server: %+v", err)
		return err
	}
	req.Header.Add("session-id", session)

	_, err = client.Do(req)
	if err != nil {
		log.Printf("Failed to stop session on server: %+v", err)
		return err
	}

	return nil
}

func CheckFinished(session string) error {
	// Continuously or long poll server for session finished state
	// On success it will write back final pose graph
	// We write posegraph to pg file
	return nil
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

	pose_session_dir := get_session_dir(session, POSE)
	if err := os.MkdirAll(pose_session_dir, 0777); err != nil {
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
