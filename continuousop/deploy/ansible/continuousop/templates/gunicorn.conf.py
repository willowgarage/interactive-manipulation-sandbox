bind = '127.0.0.1:{{ gunicorn_port }}'
logfile = '{{ webapps_dir }}/{{ app_name }}/log/gunicorn.log'
workers = 2
