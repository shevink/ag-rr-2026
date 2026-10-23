## Zusatz: Unbekannte Hindernisse
Falls ihr eine weitere Herausforderung sucht, könnte ihr über die Launchfile `tasks <euer-launchfile>_dynamic.yaml` unbekannte Hindernisse aktivieren. Sie sind nicht auf der Karte eingetragen und treten in zwei Variationen auf: dynamisch und statisch. Neue Topics informieren euch über das Entdecken eines Hindernissen, wenn ihr in dessen Nähe seit. Die Maße der Hindernisses sind in jedem Fall fest, während sich die aktuellen Koordinaten dynamisch verändern können. Im letzteren Fall werdet ihr über die aktualisierte Position periodisch informiert, sofern ihr noch im dazu hinreichenden Abstand zum Hindernis seit.

### Zusätzliche Eingaben
- `/obstacle/static` informiert über statische Hindernisse
	- Ihr werdet im Abstand von 30cm zum Hindernis informiert 
	- Frequenz: frei konfigurierbar
- `/obstacle/dynamic` informiert über dynamische Hindernisse
	- Ihr werdet im Abstand von 30cm zum Hindernis informiert
	- Frequenz: frei konfigurierbar