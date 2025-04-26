using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TabSystem : MonoBehaviour
{
    Button[] buttons;
    List<GameObject> tabs;
    void Start()
    {
        tabs = new List<GameObject>();
        buttons = GetComponentsInChildren<Button>();
        foreach (Button button in buttons)
        {
            GameObject tab = button.transform.GetChild(0).GetChild(0).gameObject;
            button.onClick.AddListener(() => {
                ButtonClicked(tab);
            });
            tabs.Add(tab);
        }
        
        ButtonClicked(tabs[0]);
    }

    void ButtonClicked(GameObject tab){
        Debug.Log("Button clicked: " + tab.name);
        foreach (GameObject t in tabs)
        {
            if (t == tab)
            {
                t.SetActive(true);
            }
            else
            {
                t.SetActive(false);
            }
        }
    }
}
