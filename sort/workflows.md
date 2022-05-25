# obsolet - kein Nutzen?

# Neues Softwareprojekt anlegen
## Daten

#### Git-Struktur (WIP - ALTERNATIVE)
![image](uploads/19b1b9c3de51440fe8a8c8f064848190/image.png)
	
#### Projekt-Mitarbeiter
 
## Basic Flow
1.	Projektart definieren (Demonstrator, Projekt, BachelorArbeit/MasterArbeit, …)
2.	Projektgruppe mit TAG je nach Projektart erzeugen 
3.	Eingangsprojekt in Projektgruppe erzeugen
4.	Mitarbeiter einladen
5.	Mitarbeiterrollen verteilen
6.	Ankündigung der fertig angelegten Git-Struktur
 
## Alternative Flows
1. Gruppenicon hochladen
2. Gruppenmitglieder Icon hochladen lassen
3. Projektziele festlegen & dokumentieren
4. Projekt-Architektur festlegen & dokumentieren
5. Projekt-Use-Cases festlegen & dokumentieren
6. Detail-Dokumentation erzeugen
7. Projekte für Sub-Komponenten der Architektur erzeugen
8. Weiterführende Informationen verlinken (Teams Projekt)
9. Bei Demo-Projekten mit Demonstrator Product-Owner absprechen
#

# Bestehendes Softwareprojekt verändern, um dieses um Funktionen zu erweitern oder zu verbessern

## Daten
 `<Ausgewähltes Projekt>`

 `<Projekt-Betreuer>`

 `<Use-Case Draft für Feature>`

## Basic Flow
1. Notwendigkeit des Features klarstellen
2. Projekt-Betreuer kontaktieren
3. Stabile Software-Version der Software identifizieren
4. Issue für neues Feature erzeugen und beschreiben
5. Alternativ: Issue für Bug oder ähnliches erzeugen und beschreiben
6. Software-Branch für Issue/Feature erzeugen
7. Feature umsetzen
8. Feature (automatisiert) testen
9. Feature-Branch gemeinsam mit Projekt-Betreuer bzw. Projekt-Mitarbeitern integrieren & mergen
10. Änderungen auf andere Softwaresysteme dokumentieren

## Alternative Flows
1. Mitarbeiter einladen
2. Für große Features: Projekt-Use-Case ausarbeiten
3. Je nach Feature-Umfang einen Feature-Ansprechpartner wählen
4. Alternativ: Projekt zuerst forken und wieder mergen (bspw. wenn man keine Rechte vergeben möchte/kann etc.)


# Bestehendes Softwareprojekt in einem anderen Projekt verwenden (z.B. Libraries)

## Daten
`<Ausgewähltes Ziel-Projekt>`

`<Ausgewähltes Bestehendes-Projekt> (= Projekt das in einem Ziel-Projekt verwendet werden soll)`

`<Bestehendes-Projekt Betreuer>`

## Basic Flow
1. Bestehendes-Projekt Betreuer kontaktieren
2. Stabile Software-Version der Software des Bestehenden-Projekts identifizieren
3. Bestehendes-Projekt Software-Version forken? / Software-Version Taggen ?
4. Mitarbeiter einladen
5. Mitarbeiterrollen verteilen
6. Software-Version nutzen
7. Nutzung und Änderungen auf andere Softwaresysteme dokumentieren

## Alternative Flows
1. Bei Langzeitprojekten (Demo-Projekte) Minor Updates des Forks vornehmen


# Ergebnisse zweier Projekte in einem neuen Projekt zusammenfließen lassen

## Daten
`<Ausgewähltes Projekt A>`

`<Ausgewähltes Projekt B>`

`<Projekt A Betreuer>`

`<Projekt B Betreuer>`

`<Ziel des Zusammenfließens>`

## Basic Flow
1. Zweck & Kosten/Nutzen des Zusammenfließens klar machen
2. Ziele durch beide Projekt-Betreuer abstimmen
3. Erzeugen eines vollständig neuen Projekts
4. Projektverantwortlichen wählen
5. Stabile Software-Version der Software beider Projekte identifizieren, die zusammenfließen sollen
6. Stabile Software-Version Taggen
7. Issues für Integration erzeugen
8. Abhängigkeiten identifizieren & Umsetzungsplan gestalten
9. Aufräumprozess & Erarbeitung einer klaren gemeinsamen Struktur auf der die Projekte aufbauen möchten
10. Software-Branches für Issues erzeugen
11. Integrations-Features umsetzen
12. Integrations-Features (automatisiert) testen
13. Integrations-Features -Branch gemeinsam mergen
14. Dokumentationen mergen & anpassen

## Alternative Flows
1. Mitarbeiter einladen
2. Je nach Feature-Umfang, Feature-Ansprechpartner wählen


# Software aus Projekten in Demonstratoren überführen

## Daten
`<Ausgewähltes Demo-Projekt>`

`<Ausgewählte SF-Projekt Software>`

`<Demo-Projekt Betreuer>`

`<Use-Case Draft>`

## Basic Flow
1. Demo-Projekt Betreuer kontaktieren
2. Ziele durch beide Projekt-Betreuer abstimmen
3. Stabile Software-Version der Software beider Projekte identifizieren, die zusammenfließen sollen
4. Stabile Software-Version Taggen
5. Subprojekt/Use-Case Projekt in Demo-Projekt-Gruppe erzeugen
6. Issues für Integration im Demo-Projekt & Use-Case Projekt erzeugen
7. Abhängigkeiten identifizieren & Umsetzungsplan gestalten
8. Software-Branches für zu integrierende SF-Software-Komponente erzeugen
9. SF-Software-Komponente integrieren
10. SF-Software-Komponente (automatisiert) testen
11. SF-Software-Komponente Branch gemeinsam mergen
12. Dokumentationen mergen & anpassen
 

## Alternative Flows
1. Mitarbeiter einladen
2. Je nach SF-Software-Komponenten Umfang, einen verantwortlichen Maintainer festlegen

#