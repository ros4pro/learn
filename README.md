# ROS4PRO material

## [Access this content online](http://learn.ros4.pro)

## Compile in development

```bash
npm install gitbook-cli -g
gitbook install
gitbook serve
```

## Compile the resources manually

```bash
npm install gitbook-cli -g
gitbook install
gitbook build
```

Then serve and load the boook with your favorite static web server and browser:

```bash
cd _book
python -m http.server
firefox http://localhost:8000
```

### USE jupyter compilation

I use pyenv with python 3.7.2

1. Install nbinteract (0.1.7) and other dependencies

```bash
pip install nbinteract==0.1.7 nbconvert==5.3.1 tornado==4.2
```

2. Put your jupyter notebook in the textbook folder

3. Launch python script `convert_notebooks_to_html_partial.py`
```bash
python convert_notebooks_to_html_partial.py
```

4. Insert in your `README.md`, the path to the html document on your notebook (include in `notebooks-html`) :
```js
{% include "../../notebooks-html/COVID-19.html" %}
```

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No. 732287.
