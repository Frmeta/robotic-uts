using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class Timer : MonoBehaviour
{
    public Scrollbar timeMultipliercrollbar;
    public TMP_Text timerText;
    public float currTime = 0f;

    public static Timer instance;

    void Awake(){
        if(instance == null){
            instance = this;
        } else {
            Destroy(gameObject);
        }
    }

    void Start(){
        timeMultipliercrollbar.onValueChanged.AddListener( (value) =>{
            Time.timeScale = Mathf.Lerp(0.2f, 15f, value);
        }   
        );
        StartCoroutine(StartTimer());
    }

    IEnumerator StartTimer(){
        while(true){
            yield return new WaitForSeconds(1);
            currTime++;

            float minutes = Mathf.Floor(currTime / 60);
            float seconds = currTime % 60;

            timerText.text = "Time elapsed: " + minutes.ToString("00") + ":" + seconds.ToString("00");
        }
    }

    public void StopTimer(){
        StopAllCoroutines();
    }

    void Update(){
        if(Input.GetKeyDown(KeyCode.R)){
            UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
        }
    }
}
