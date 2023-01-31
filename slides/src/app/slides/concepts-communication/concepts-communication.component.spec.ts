import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConceptsCommunicationComponent } from './concepts-communication.component';

describe('ConceptsCommunicationComponent', () => {
  let component: ConceptsCommunicationComponent;
  let fixture: ComponentFixture<ConceptsCommunicationComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConceptsCommunicationComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConceptsCommunicationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
